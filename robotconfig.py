"""Robot configuration, the way you'd set one up on a Driver Hub.

On a real robot you don't hard-code which motor is which. You open the
configuration screen, and for each hub port you say what's plugged in, what to
call it, and whether it's reversed. Then your OpMode asks for it by name:

    hardwareMap.get(DcMotor.class, "front_left")

This does the same thing. After importing a robot from CAD its joints have
whatever names the mates had, and a config maps those onto a drivetrain layout
the code understands -- so `wheel_fl_joint_v2` can be your front-left drive
motor without renaming anything.

The reversed flag matters more than it looks. On any real drivetrain, the
motors on one side face the opposite way, so commanding "forward" spins them
backwards. Every FTC team sets `setDirection(REVERSE)` on one side. Get it wrong
and the robot spins in place instead of driving -- which is exactly the bug this
field exists to make explicit rather than mysterious.
"""

import json
from dataclasses import dataclass, field, asdict
from pathlib import Path

import numpy as np

# REV hubs: four motor ports each, six servo ports each.
MOTOR_PORTS = [f"{hub}{i}" for hub in ("CH", "EH") for i in range(4)]
SERVO_PORTS = [f"{hub}S{i}" for hub in ("CH", "EH") for i in range(6)]

DRIVETRAINS = {
    "mecanum4": {
        "label": "4-wheel Mecanum  (strafes sideways)",
        "positions": ["fl", "fr", "bl", "br"],
        "names": ["front left", "front right", "back left", "back right"],
        "strafes": True,
    },
    "tank4": {
        "label": "4-wheel Tank / skid steer",
        "positions": ["fl", "fr", "bl", "br"],
        "names": ["front left", "front right", "back left", "back right"],
        "strafes": False,
    },
    "tank6": {
        "label": "6-wheel Tank / skid steer",
        "positions": ["fl", "fr", "ml", "mr", "bl", "br"],
        "names": ["front left", "front right", "middle left", "middle right",
                  "back left", "back right"],
        "strafes": False,
    },
    "tank2": {
        "label": "2-wheel Tank",
        "positions": ["l", "r"],
        "names": ["left", "right"],
        "strafes": False,
    },
}


@dataclass
class MotorEntry:
    """One motor: where it's plugged in, what it drives, which way round."""
    port: str
    name: str
    joint: str
    reversed: bool = False
    motor_type: str = "YJ_312"

    def check(self, valid_joints):
        errs = []
        if self.port not in MOTOR_PORTS:
            errs.append(f"{self.name}: port {self.port!r} isn't a real port "
                        f"(expected one of {', '.join(MOTOR_PORTS)})")
        if self.joint not in valid_joints:
            errs.append(f"{self.name}: joint {self.joint!r} isn't in the model")
        return errs


@dataclass
class RobotConfig:
    drivetrain: str = "mecanum4"
    drive: dict = field(default_factory=dict)      # position -> MotorEntry
    mechanisms: dict = field(default_factory=dict)  # name -> MotorEntry
    wheel_preset: str = "gobilda_96"

    # -- persistence -----------------------------------------------------

    def save(self, path):
        data = {
            "drivetrain": self.drivetrain,
            "wheel_preset": self.wheel_preset,
            "drive": {k: asdict(v) for k, v in self.drive.items()},
            "mechanisms": {k: asdict(v) for k, v in self.mechanisms.items()},
        }
        Path(path).write_text(json.dumps(data, indent=2) + "\n")

    @classmethod
    def load(cls, path):
        d = json.loads(Path(path).read_text())
        return cls(
            drivetrain=d.get("drivetrain", "mecanum4"),
            wheel_preset=d.get("wheel_preset", "gobilda_96"),
            drive={k: MotorEntry(**v) for k, v in d.get("drive", {}).items()},
            mechanisms={k: MotorEntry(**v) for k, v in d.get("mechanisms", {}).items()},
        )

    # -- validation ------------------------------------------------------

    def validate(self, valid_joints):
        """Return a list of problems. Empty means the config is usable."""
        errs = []
        if self.drivetrain not in DRIVETRAINS:
            return [f"unknown drivetrain {self.drivetrain!r}; "
                    f"options: {', '.join(DRIVETRAINS)}"]

        spec = DRIVETRAINS[self.drivetrain]
        for pos in spec["positions"]:
            if pos not in self.drive:
                errs.append(f"{spec['label']} needs a motor for position {pos!r}")

        seen_ports, seen_joints = {}, {}
        for entry in list(self.drive.values()) + list(self.mechanisms.values()):
            errs += entry.check(valid_joints)
            # Two motors on one port is physically impossible; two motors on one
            # joint means you configured the same wheel twice.
            if entry.port in seen_ports:
                errs.append(f"port {entry.port} used twice: "
                            f"{seen_ports[entry.port]} and {entry.name}")
            seen_ports[entry.port] = entry.name
            if entry.joint in seen_joints:
                errs.append(f"joint {entry.joint} driven twice: "
                            f"{seen_joints[entry.joint]} and {entry.name}")
            seen_joints[entry.joint] = entry.name
        return errs

    # -- drivetrain maths ------------------------------------------------

    @property
    def positions(self):
        return DRIVETRAINS[self.drivetrain]["positions"]

    @property
    def strafes(self):
        return DRIVETRAINS[self.drivetrain]["strafes"]

    def mix(self, fwd, strafe, turn):
        """Driver inputs -> one power per drive motor, in `positions` order.

        Reversal is applied here, so everything downstream can pretend every
        motor is mounted the same way round.
        """
        if self.drivetrain == "mecanum4":
            raw = {
                "fl": fwd + strafe + turn,
                "fr": fwd - strafe - turn,
                "bl": fwd - strafe + turn,
                "br": fwd + strafe - turn,
            }
        else:
            # Skid steer: strafe does nothing, both sides differ by the turn.
            left, right = fwd + turn, fwd - turn
            raw = {p: (left if p.endswith("l") else right) for p in self.positions}

        p = np.array([raw[pos] for pos in self.positions], dtype=float)
        peak = np.abs(p).max()
        if peak > 1.0:
            p /= peak
        for i, pos in enumerate(self.positions):
            if self.drive[pos].reversed:
                p[i] = -p[i]
        return p

    def summary(self):
        spec = DRIVETRAINS[self.drivetrain]
        out = [f"Drivetrain: {spec['label']}",
               f"Wheels:     {self.wheel_preset}", "",
               f"  {'port':<6} {'name':<16} {'joint':<20} {'dir':<9} motor"]
        out.append("  " + "-" * 62)
        for pos in self.positions:
            e = self.drive.get(pos)
            if e is None:
                out.append(f"  {'--':<6} {pos:<16} {'(unassigned)':<20}")
                continue
            out.append(f"  {e.port:<6} {e.name:<16} {e.joint:<20} "
                       f"{'REVERSED' if e.reversed else 'forward':<9} {e.motor_type}")
        if self.mechanisms:
            out.append("")
            for e in self.mechanisms.values():
                out.append(f"  {e.port:<6} {e.name:<16} {e.joint:<20} "
                           f"{'REVERSED' if e.reversed else 'forward':<9} {e.motor_type}")
        if not self.strafes:
            out.append("")
            out.append("  NOTE: this drivetrain cannot strafe; sideways input is ignored.")
        return "\n".join(out)


def default_config():
    """Matches the built-in robot, so nothing needs a config file to run."""
    ports = ["CH0", "CH1", "CH2", "CH3"]
    names = ["front_left", "front_right", "back_left", "back_right"]
    return RobotConfig(
        drivetrain="mecanum4",
        drive={pos: MotorEntry(port=ports[i], name=names[i],
                               joint=f"drive_{pos}", reversed=False)
               for i, pos in enumerate(["fl", "fr", "bl", "br"])},
        mechanisms={"arm": MotorEntry(port="EH0", name="arm",
                                      joint="shoulder", motor_type="YJ_60")},
    )
