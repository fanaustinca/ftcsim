"""Interactive robot configuration, like the Driver Hub's config screen.

    python configure.py                        # configure the built-in robot
    python configure.py imported.xml           # configure a CAD import
    python configure.py imported.xml -o my.json

Walks the same steps you'd do on a Driver Hub: pick your drivetrain, then for
each wheel say which port it's on, what it's called, and whether it's reversed.
The difference is that here it also has to map onto the joints in the model --
after a CAD import those are named after your Onshape mates, not after anything
the code expects.
"""
import argparse
import sys

import mujoco
import numpy as np

import robotconfig as rc
import wheels
from robotconfig import RobotConfig, MotorEntry, DRIVETRAINS, MOTOR_PORTS


def model_joints(path=None):
    if path:
        model = mujoco.MjModel.from_xml_path(path)
    else:
        from robot import build_mjcf
        model = mujoco.MjModel.from_xml_string(build_mjcf())
    out = []
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if not name or name.startswith("rollerj_"):
            continue          # roller joints are internal to the wheels
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            continue
        out.append(name)
    return out


def choose(prompt, options, labels=None, default=None):
    labels = labels or options
    print(f"\n{prompt}")
    for i, lab in enumerate(labels, 1):
        mark = "  (default)" if options[i - 1] == default else ""
        print(f"  {i}. {lab}{mark}")
    while True:
        raw = input("  > ").strip()
        if not raw and default is not None:
            return default
        if raw.isdigit() and 1 <= int(raw) <= len(options):
            return options[int(raw) - 1]
        print(f"  enter 1-{len(options)}")


def ask(prompt, default=""):
    raw = input(f"{prompt}{f' [{default}]' if default else ''}: ").strip()
    return raw or default


def yes_no(prompt, default=False):
    d = "Y/n" if default else "y/N"
    while True:
        raw = input(f"{prompt} ({d}): ").strip().lower()
        if not raw:
            return default
        if raw in ("y", "yes"):
            return True
        if raw in ("n", "no"):
            return False


def auto_config(joints, drivetrain="mecanum4"):
    """Best-effort guess, for when you'd rather edit JSON than answer prompts.

    Matches joints to wheel positions by looking for the position code as a
    word in the joint name -- `dof_drive_fl`, `wheel_FL_hinge` and `fl_drive`
    all match "fl". Ports are assigned in order, and right-side motors are
    marked reversed because that is true on essentially every drivetrain.

    It guesses. Always read the summary before trusting it.
    """
    spec = DRIVETRAINS[drivetrain]
    cfg = RobotConfig(drivetrain=drivetrain)
    used = []
    # Ports go by how many motors we've actually assigned, not by position
    # index -- otherwise a gap (an unmatched wheel) leaves a port index unused
    # and the mechanism loop below reuses it.
    next_port = 0

    for pos in spec["positions"]:
        match = next((j for j in joints
                      if j not in used
                      and pos in j.lower().replace("-", "_").split("_")), None)
        if match is None:
            # Leave it unassigned rather than grabbing an unrelated joint.
            # Grabbing whatever was free assigned `shoulder` and `wrist` as
            # drive motors on a 6WD config, which validate() would never have
            # caught because the slots looked filled.
            continue
        used.append(match)
        cfg.drive[pos] = MotorEntry(
            port=MOTOR_PORTS[next_port], name=f"{pos}_drive", joint=match,
            reversed=pos.endswith("r"), motor_type="YJ_312")
        next_port += 1

    port_i = next_port
    for j in joints:
        if j in used or port_i >= len(MOTOR_PORTS):
            continue
        # Servo-ish joints (grippers, wrists) aren't motors; skip them.
        if any(t in j.lower() for t in ("grip", "wrist", "servo", "claw")):
            continue
        cfg.mechanisms["arm" if "shoulder" in j.lower() or "arm" in j.lower() else j] = \
            MotorEntry(port=MOTOR_PORTS[port_i], name=j, joint=j, motor_type="YJ_60")
        port_i += 1
    return cfg


def detect_reversal(cfg, model_path=None):
    """Work out which motors are wired backwards by driving them.

    Whether a motor needs reversing depends on which way it is physically
    mounted, which after a CAD import comes from the mate axis. Guessing from
    the name ("right side is usually reversed") is right on real hardware and
    wrong on a model whose joints all share an axis -- so instead of guessing,
    spin each motor on its own and watch which way the robot goes.

    A drive motor given positive power should move the robot forwards. If it
    moves backwards, that motor is reversed.
    """
    import mujoco
    from motors import Robot
    import copy

    if model_path:
        model = mujoco.MjModel.from_xml_path(model_path)
    else:
        from robot import build_mjcf
        model = mujoco.MjModel.from_xml_string(build_mjcf(wheel_preset=cfg.wheel_preset))

    probe = copy.deepcopy(cfg)
    for e in probe.drive.values():
        e.reversed = False

    results = {}
    for i, pos in enumerate(probe.positions):
        data = mujoco.MjData(model)
        bot = Robot(model, data, config=probe)
        for _ in range(400):
            bot.apply()
            mujoco.mj_step(model, data)
        x0 = bot.pose()[0]
        for _ in range(700):
            bot.drive_power = np.zeros(len(probe.positions))
            bot.drive_power[i] = 1.0
            bot.apply()
            mujoco.mj_step(model, data)
        results[pos] = bot.pose()[0] - x0
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("model", nargs="?", help="MJCF file (omit for the built-in robot)")
    ap.add_argument("-o", "--output", default="robot_config.json")
    ap.add_argument("--auto", action="store_true",
                    help="guess everything and write it out, no prompts")
    ap.add_argument("--drivetrain", default="mecanum4", choices=list(DRIVETRAINS))
    ap.add_argument("--no-detect", action="store_true",
                    help="skip measuring motor directions; guess from names")
    a = ap.parse_args()

    joints = model_joints(a.model)
    if not joints:
        print("No usable joints found in the model.")
        return 1

    print("=" * 66)
    print("ROBOT CONFIGURATION")
    print("=" * 66)
    print(f"\nModel: {a.model or 'built-in robot'}")
    print(f"Found {len(joints)} joints you can assign a motor to:")
    for j in joints:
        print(f"  - {j}")

    if a.auto:
        cfg = auto_config(joints, a.drivetrain)
        if not a.no_detect and len(cfg.drive) == len(DRIVETRAINS[a.drivetrain]["positions"]):
            print("\nMeasuring motor directions (driving each one alone)...")
            moved = detect_reversal(cfg, a.model)
            for pos, dx in moved.items():
                cfg.drive[pos].reversed = dx < 0
                print(f"  {pos:<4} moves robot {dx:+.3f} m  -> "
                      f"{'REVERSED' if dx < 0 else 'forward'}")
        print("\n" + cfg.summary())
        errs = cfg.validate(joints)
        if errs:
            print("\nPROBLEMS:")
            for e in errs:
                print(f"  - {e}")
        cfg.save(a.output)
        print(f"\nSaved to {a.output}  (guessed -- check it before trusting it)")
        return 1 if errs else 0

    keys = list(DRIVETRAINS)
    dt = choose("What drivetrain does this robot have?",
                keys, [DRIVETRAINS[k]["label"] for k in keys], default="mecanum4")
    spec = DRIVETRAINS[dt]

    if not spec["strafes"]:
        print("\n  Note: this drivetrain cannot strafe. Sideways stick input")
        print("  will be ignored, the same as on the real thing.")

    preset = choose("Which wheels?", list(wheels.PRESETS),
                    [wheels.PRESETS[k].name for k in wheels.PRESETS],
                    default=wheels.DEFAULT)

    print("\n" + "-" * 66)
    print("Now assign each drive motor. This is the part you'd do on the")
    print("Driver Hub: which port, what name, and which way round.")
    print("-" * 66)

    cfg = RobotConfig(drivetrain=dt, wheel_preset=preset)
    used_ports, used_joints = [], []

    for pos, human in zip(spec["positions"], spec["names"]):
        print(f"\n  === {human.upper()} ===")

        free_joints = [j for j in joints if j not in used_joints]
        # A joint whose name contains the position code is almost certainly the
        # right one, so offer it first.
        guess = next((j for j in free_joints if pos in j.lower().split("_")), None)
        joint = choose(f"  Which joint is the {human} wheel?", free_joints,
                       default=guess)
        used_joints.append(joint)

        free_ports = [p for p in MOTOR_PORTS if p not in used_ports]
        port = choose(f"  Which port is it plugged into?", free_ports,
                      default=free_ports[0])
        used_ports.append(port)

        name = ask(f"  Name it", human.replace(" ", "_"))

        # Right-side motors face the other way on essentially every drivetrain.
        suggest_rev = pos.endswith("r")
        rev = yes_no(f"  Reversed?", default=suggest_rev)
        if suggest_rev and not rev:
            print("    (heads up: right-side motors are usually reversed)")

        cfg.drive[pos] = MotorEntry(port=port, name=name, joint=joint,
                                    reversed=rev, motor_type="YJ_312")

    # Anything left over is a mechanism.
    leftover = [j for j in joints if j not in used_joints]
    if leftover and yes_no(f"\nConfigure {len(leftover)} remaining joint(s) as "
                           f"mechanisms?", default=True):
        for joint in leftover:
            print(f"\n  === {joint} ===")
            if not yes_no(f"  Is {joint} driven by a motor?", default=True):
                continue
            free_ports = [p for p in MOTOR_PORTS if p not in used_ports]
            if not free_ports:
                print("  No ports left (8 motors is the hub limit).")
                break
            port = choose("  Which port?", free_ports, default=free_ports[0])
            used_ports.append(port)
            name = ask("  Name it", joint)
            mtype = choose("  Which motor?", ["YJ_312", "YJ_60"],
                           ["goBILDA 312 RPM (fast)", "goBILDA 60 RPM (strong)"],
                           default="YJ_60")
            rev = yes_no("  Reversed?", default=False)
            cfg.mechanisms[name] = MotorEntry(port=port, name=name, joint=joint,
                                              reversed=rev, motor_type=mtype)

    print("\n" + "=" * 66)
    print(cfg.summary())
    print("=" * 66)

    errs = cfg.validate(joints)
    if errs:
        print("\nPROBLEMS:")
        for e in errs:
            print(f"  - {e}")
        if not yes_no("\nSave anyway?", default=False):
            print("Not saved.")
            return 1

    cfg.save(a.output)
    print(f"\nSaved to {a.output}")
    print("\nUse it with:")
    print(f"    python teleop.py --config {a.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
