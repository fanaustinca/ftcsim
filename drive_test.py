"""Drivetrain validation, now going through the real motor model.

Checks that each of the three mecanum motions goes where it should, and that
top speed is physically plausible rather than 7000 RPM of wheelspin.
"""
import mujoco
import numpy as np

from robot import build_mjcf
from motors import Robot, YJ_312, WHEELS

model = mujoco.MjModel.from_xml_string(build_mjcf())


def run(fwd, strafe, turn, seconds=2.0):
    data = mujoco.MjData(model)
    bot = Robot(model, data)
    bot.stow()
    bot.set_grip(0.0)

    for _ in range(500):            # settle onto the wheels
        bot.hold_arm(1.2)
        bot.apply()
        mujoco.mj_step(model, data)

    x0, y0, h0 = bot.pose()
    bot.heading_target = None

    peak_rads = 0.0
    for _ in range(int(seconds / model.opt.timestep)):
        # drive_held is a feedback controller, so it has to run every step --
        # setting it once before the loop (as this used to) just latched a
        # fixed wheel command and the heading correction never happened.
        bot.drive_held(fwd, strafe, turn)
        bot.hold_arm(1.2)
        bot.apply()
        mujoco.mj_step(model, data)
        peak_rads = max(peak_rads, max(abs(data.qvel[d]) for d in bot._drive_dof))

    x1, y1, h1 = bot.pose()
    dh = np.degrees((h1 - h0 + np.pi) % (2 * np.pi) - np.pi)
    dist = np.hypot(x1 - x0, y1 - y0)
    return x1 - x0, y1 - y0, dh, dist / seconds, peak_rads


print(f"free speed = {YJ_312.free_rads:.1f} rad/s "
      f"-> theoretical top speed {YJ_312.free_rads * 0.048:.2f} m/s\n")

print(f"{'command':<15} {'dX':>7} {'dY':>7} {'dYaw':>7} {'m/s':>7} {'peak rad/s':>11}")
print("-" * 60)
for name, cmd in [
    ("forward",      (1, 0, 0)),
    ("backward",     (-1, 0, 0)),
    ("strafe left",  (0, -1, 0)),
    ("strafe right", (0, 1, 0)),
    ("turn ccw",     (0, 0, 1)),
    ("diagonal",     (1, 1, 0)),
]:
    dx, dy, dh, spd, pk = run(*cmd)
    print(f"{name:<15} {dx:>7.2f} {dy:>7.2f} {dh:>7.1f} {spd:>7.2f} {pk:>11.1f}")
