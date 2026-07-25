"""Search for a claw pose that actually picks the cube up.

Three coupled parameters -- how close the robot stops, how far the shoulder
drops, and how far the wrist rotates the claw over -- so rather than solving it
by hand we just try a grid and see which combinations lift the cube.

This is a legitimate way to use a simulator: it's a design question ("can this
arm geometry pick that up, and from where?") answered by running the physics
instead of by arguing about it.
"""
import itertools

import mujoco
import numpy as np

from robot import build_mjcf
from motors import Robot

model = mujoco.MjModel.from_xml_string(build_mjcf())
CUBE_START_Z = 0.038


def attempt(stop_x, shoulder, wrist_off, verbose=False):
    data = mujoco.MjData(model)
    bot = Robot(model, data)
    bot.stow()

    def run(n, arm, grip, goto=None):
        for _ in range(n):
            if goto is not None:
                err = goto - bot.pose()[0]
                bot.drive(float(np.clip(2.5 * err, -0.6, 0.6)), 0, 0)
            else:
                bot.drive(0, 0, 0)
            bot.hold_arm(arm)
            bot.level_wrist(wrist_off)
            bot.set_grip(grip)
            bot.apply()
            mujoco.mj_step(model, data)

    run(400, 1.2, 0.0)                      # settle, claw open, arm up
    run(1100, 1.2, 0.0, goto=stop_x)        # approach with arm clear
    run(900, shoulder, 0.0, goto=stop_x)    # lower over the cube
    run(700, shoulder, 1.0, goto=stop_x)    # close
    run(900, 1.0, 1.0, goto=stop_x)         # lift

    lift = float(data.body("cube").xpos[2]) - CUBE_START_Z
    if verbose:
        print(f"  cube z = {data.body('cube').xpos[2]:.3f} (lift {lift:+.3f})")
    return lift


best = None
print(f"{'stop_x':>7} {'shoulder':>9} {'wrist_off':>10} {'lift (m)':>9}")
print("-" * 40)

for stop_x, shoulder, wrist_off in itertools.product(
    [0.55, 0.62, 0.69],
    [-0.40, -0.25, -0.10],
    [1.45, 1.10, 0.75],
):
    lift = attempt(stop_x, shoulder, wrist_off)
    flag = "  <-- LIFTED" if lift > 0.05 else ""
    print(f"{stop_x:>7.2f} {shoulder:>9.2f} {wrist_off:>10.2f} {lift:>9.3f}{flag}")
    if best is None or lift > best[0]:
        best = (lift, stop_x, shoulder, wrist_off)

print("-" * 40)
print(f"best: lift={best[0]:.3f} at stop_x={best[1]} shoulder={best[2]} wrist_off={best[3]}")
