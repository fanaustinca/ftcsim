"""Motor model + a robot API that looks roughly like the FTC SDK.

The important part is `Motor.torque()`. A real DC motor cannot produce stall
torque at full speed -- as it spins up, back-EMF eats the available torque
until, at free speed, it produces none at all. MuJoCo's `motor` actuator has no
such limit: ask for 3 N*m and it gives you 3 N*m at 8000 RPM, which is how you
end up with a sim that says every mechanism works.

The model is the standard linear one:

    T = T_stall * (power - omega / omega_free)

At power=1, omega=0 you get stall torque. At power=1, omega=omega_free you get
zero. At power=0 with the motor still spinning you get negative torque, which is
the braking you feel when you release the sticks with ZeroPowerBehavior.BRAKE.
"""

from dataclasses import dataclass, replace

import mujoco
import numpy as np

import robotconfig as rc


@dataclass(frozen=True)
class Motor:
    """Specs are at the OUTPUT shaft, i.e. after the gearbox.

    Numbers below are ballpark goBILDA 5203 Yellow Jacket figures. Check them
    against the actual product page before trusting a result -- the whole point
    of this model is using real numbers.
    """
    name: str
    free_rpm: float
    stall_nm: float
    ticks_per_rev: float
    efficiency: float = 0.85

    # External reduction BETWEEN the motor and the mechanism: chain and
    # sprockets, a belt, a gear pair. 3.0 means a 3:1 reduction -- three times
    # the torque at a third of the speed.
    #
    # This is separate from the gearbox already inside the motor: goBILDA quote
    # their figures at the output shaft, so a 19.2:1 Yellow Jacket's planetary
    # is already folded into stall_nm and free_rpm. Anything you add outside the
    # motor goes here.
    #
    # CAD import does NOT give you this. The exporter sees two rotating parts
    # and produces two independent joints; nothing in the mate tells it they are
    # coupled by a chain. You have to state the ratio yourself.
    external_ratio: float = 1.0
    external_efficiency: float = 0.95      # ~0.95 chain, ~0.97 belt, ~0.90 gears

    @property
    def free_rads(self) -> float:
        """Free speed at the MECHANISM, after any external reduction."""
        return self.free_rpm * 2 * np.pi / 60.0 / self.external_ratio

    @property
    def peak_nm(self) -> float:
        """Stall torque at the MECHANISM, after any external reduction."""
        return self.stall_nm * self.external_ratio * self.external_efficiency

    def torque(self, power: float, omega: float) -> float:
        power = float(np.clip(power, -1.0, 1.0))
        peak = self.peak_nm
        t = peak * (power - omega / self.free_rads)
        # A motor can't produce more than stall torque in either direction.
        t = float(np.clip(t, -peak, peak))
        return t * self.efficiency

    def geared(self, ratio: float, efficiency: float = 0.95) -> "Motor":
        """This motor with an external reduction bolted on.

            YJ_312.geared(2.0)   # 2:1 chain to the wheels
        """
        return replace(self, external_ratio=ratio, external_efficiency=efficiency)

    def summary(self) -> str:
        return (f"{self.name}"
                + (f" + {self.external_ratio:g}:1 external" if self.external_ratio != 1 else "")
                + f"  ->  {self.free_rads * 60 / (2 * np.pi):.0f} rpm, "
                  f"{self.peak_nm * self.efficiency:.2f} N*m at the mechanism")


# 19.2:1, the standard FTC drivetrain choice.
YJ_312 = Motor("goBILDA 5203 312rpm", free_rpm=312, stall_nm=2.4, ticks_per_rev=537.7)
# 99.5:1, geared way down for arms -- slow and strong.
YJ_60 = Motor("goBILDA 5203 60rpm", free_rpm=60, stall_nm=7.5, ticks_per_rev=2786.2)

WHEELS = ["fl", "fr", "bl", "br"]


def mecanum(fwd: float, strafe: float, turn: float) -> np.ndarray:
    """Mix driver inputs into four wheel powers, normalising so nothing clips.

    Sign conventions match the standard FTC teleop snippet, so this maps
    straight onto gamepad axes with no negation:
        fwd    +1 = forward
        strafe +1 = right   (gamepad1.left_stick_x)
        turn   +1 = clockwise / right  (gamepad1.right_stick_x)

    Scaling by the max magnitude (rather than clipping each wheel) keeps the
    ratios between wheels intact, so the robot still travels in the commanded
    direction at full stick instead of curving.
    """
    p = np.array([
        fwd + strafe + turn,   # fl
        fwd - strafe - turn,   # fr
        fwd - strafe + turn,   # bl
        fwd + strafe - turn,   # br
    ])
    peak = np.abs(p).max()
    return p / peak if peak > 1.0 else p


class Robot:
    """Thin wrapper over MjModel/MjData exposing FTC-shaped controls."""

    def __init__(self, model, data, battery_v: float = 12.5, config=None):
        """config: a RobotConfig, or None for the built-in mecanum layout.

        The config is what lets an imported robot work: after a CAD export the
        joints are named after your Onshape mates, so the mapping from "front
        left drive motor" to an actual joint has to come from somewhere.
        """
        self.m, self.d = model, data
        self.battery_v = battery_v
        self.config = config or rc.default_config()

        self._motor_specs = {"YJ_312": YJ_312, "YJ_60": YJ_60}

        self._drive_j = [model.joint(self.config.drive[p].joint).id
                         for p in self.config.positions]
        self._drive_dof = [model.jnt_dofadr[j] for j in self._drive_j]
        self._drive_specs = [self._motor_specs.get(self.config.drive[p].motor_type, YJ_312)
                             for p in self.config.positions]
        self._drive_actuators = [self._actuator_for(self.config.drive[p].joint)
                                 for p in self.config.positions]

        arm = self.config.mechanisms.get("arm")
        self._shoulder_j = model.joint(arm.joint if arm else "shoulder").id
        self._shoulder_dof = model.jnt_dofadr[self._shoulder_j]
        self._arm_spec = (self._motor_specs.get(arm.motor_type, YJ_60) if arm else YJ_60)
        self._arm_actuator = self._actuator_for(arm.joint if arm else "shoulder")

        self.drive_power = np.zeros(len(self._drive_j))
        self.arm_power = 0.0
        self._encoder_zero = np.zeros(len(self._drive_j))
        self.heading_target = None      # latched heading for drive_held()

    def _actuator_for(self, joint_name):
        """Find the actuator driving a joint, whatever it happens to be called."""
        jid = self.m.joint(joint_name).id
        for a in range(self.m.nu):
            if self.m.actuator_trnid[a][0] == jid:
                return a
        raise KeyError(f"no actuator drives joint {joint_name!r}")

    # -- outputs ---------------------------------------------------------

    def drive(self, fwd, strafe, turn):
        self.drive_power = self.config.mix(fwd, strafe, turn)

    def drive_held(self, fwd, strafe, turn, kp=2.5, deadband=0.05):
        """Drive with gyro heading-hold: the robot keeps the heading you left
        it on until you actively command a turn.

        Mecanum robots drift in yaw while strafing, because lateral load
        transfer unloads the wheels unevenly and the roller contact patches
        aren't perfectly matched. It gets worse the higher the centre of
        gravity -- measured here, raising the arm pivot 130 mm took strafe yaw
        from under a degree to as much as 20 over two seconds.

        The fix real teams use is exactly this: latch the heading whenever the
        turn stick is idle, and feed the error back as a turn command. It is
        also what makes long autonomous strafes repeatable.
        """
        if abs(turn) > deadband:
            self.heading_target = None          # driver is steering; let them
            self.drive_power = self.config.mix(fwd, strafe, turn)
            return

        h = self.heading()
        if self.heading_target is None:
            self.heading_target = h
        err = (self.heading_target - h + np.pi) % (2 * np.pi) - np.pi
        self.drive_power = self.config.mix(fwd, strafe, float(np.clip(-kp * err, -1, 1)))

    def set_arm(self, power):
        self.arm_power = float(np.clip(power, -1, 1))

    def hold_arm(self, target_rad, kp=4.0, kd=0.4, kf=0.30):
        """PD position hold with gravity feedforward.

        At zero power a real motor only brakes -- it does not hold position, so
        an uncommanded arm falls. Every FTC team writes some version of this.

        The kf term is the interesting part: gravity's torque on the arm scales
        with cos(angle), maximum when the arm is straight out horizontal and zero
        when it points straight up. Feeding that forward means the PD terms only
        have to correct the leftovers, so the arm doesn't sag under its own
        weight before the error term catches up.
        """
        theta = float(self.d.joint("shoulder").qpos[0])
        omega = float(self.d.qvel[self._shoulder_dof])
        ff = kf * np.cos(theta)
        self.arm_power = float(np.clip(kp * (target_rad - theta) - kd * omega + ff, -1, 1))

    def stow(self, shoulder=1.2):
        """Put the arm in a raised starting pose instead of letting it flop."""
        self.d.qpos[self.m.jnt_qposadr[self._shoulder_j]] = shoulder
        mujoco.mj_forward(self.m, self.d)

    def set_wrist(self, angle):
        self.d.ctrl[self.m.actuator("s_wrist").id] = float(np.clip(angle, -1.7, 1.7))

    def level_wrist(self, offset=0.0):
        """Keep the claw horizontal regardless of how the shoulder is angled.

        Without this the claw tilts with the arm and drives its fingertips into
        the floor when you reach down for something. Real robots fix this either
        with a parallel-bar linkage or, as here, a wrist servo that tracks the
        shoulder. The shoulder turns about -Y and the wrist about +Y, so the two
        cancel when their angles are equal.
        """
        theta = float(self.d.joint("shoulder").qpos[0])
        self.set_wrist(theta + offset)

    def set_grip(self, closed):
        """closed: 0.0 = wide open, 1.0 = fully closed.

        Positive grip_l swings the left finger outward (its tip moves to +y),
        so OPEN is the positive end for the left finger and the negative end for
        the right one. Getting this backwards silently inverts the claw.
        """
        c = float(np.clip(closed, 0, 1))
        self.d.ctrl[self.m.actuator("s_grip_l").id] = 0.40 - c * 0.75
        self.d.ctrl[self.m.actuator("s_grip_r").id] = -0.40 + c * 0.75

    # -- inputs ----------------------------------------------------------

    def encoders(self):
        """Wheel positions in encoder ticks, like getCurrentPosition()."""
        pos = np.array([self.d.qpos[self.m.jnt_qposadr[j]] for j in self._drive_j])
        ticks = np.array([s.ticks_per_rev for s in self._drive_specs])
        return ((pos / (2 * np.pi)) * ticks - self._encoder_zero).astype(int)

    def reset_encoders(self):
        pos = np.array([self.d.qpos[self.m.jnt_qposadr[j]] for j in self._drive_j])
        ticks = np.array([s.ticks_per_rev for s in self._drive_specs])
        self._encoder_zero = (pos / (2 * np.pi)) * ticks

    def heading(self):
        """Robot yaw in radians, like the IMU's getRobotYawPitchRollAngles()."""
        q = self.d.body("chassis").xquat
        return float(np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]),
                                1 - 2 * (q[2] ** 2 + q[3] ** 2)))

    def pose(self):
        p = self.d.body("chassis").xpos
        return float(p[0]), float(p[1]), self.heading()

    # -- called every physics step --------------------------------------

    def apply(self):
        """Convert power commands into torques using the motor curve.

        Call this immediately before every mj_step.
        """
        sag = self.battery_v / 12.5   # crude, but catches "everything slows under load"

        for i in range(len(self._drive_dof)):
            omega = self.d.qvel[self._drive_dof[i]]
            tau = self._drive_specs[i].torque(self.drive_power[i], omega) * sag
            self.d.ctrl[self._drive_actuators[i]] = tau

        omega = self.d.qvel[self._shoulder_dof]
        self.d.ctrl[self._arm_actuator] = self._arm_spec.torque(self.arm_power, omega) * sag
