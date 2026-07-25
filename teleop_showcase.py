"""Renders a video of teleop-style driving -- what it looks like when driven.

Feeds a scripted stick sequence through exactly the same code path teleop.py
uses, so this is a real recording of the sim, not an animation.
"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
import imageio.v2 as imageio

from robot import build_mjcf
from motors import Robot

model = mujoco.MjModel.from_xml_string(build_mjcf())
data = mujoco.MjData(model)
bot = Robot(model, data)
bot.stow()

renderer = mujoco.Renderer(model, height=720, width=1280)
cam = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam)
cam.elevation, cam.distance = -22, 2.1

FPS = 50
frames = []

# (label, seconds, fwd, strafe, turn, arm_target)
# Short runs with a brief stop between each, so momentum from one manoeuvre
# doesn't corrupt the next -- and so nothing reaches the wall.
_ = lambda: ("stop", 0.45, 0, 0, 0, 1.2)
SCRIPT = [
    ("settle",            0.6,  0, 0, 0, 1.2),
    ("forward",           0.9,  1, 0, 0, 1.2),   _(),
    ("back",              0.9, -1, 0, 0, 1.2),   _(),
    ("strafe right",      0.9,  0, 1, 0, 1.2),   _(),
    ("strafe left",       0.9,  0, -1, 0, 1.2),  _(),
    ("spin in place",     1.5,  0, 0, 1, 1.2),   _(),
    ("diagonal",          0.9,  1, 1, 0, 1.2),   _(),
    ("arc while turning", 1.4,  1, 0, -0.5, 1.2), _(),
    ("lower the arm",     1.5,  0, 0, 0, 0.0),
    ("raise the arm",     1.5,  0, 0, 0, 1.6),
    ("drive with arm up", 1.0, -1, 0, 0, 1.6),
]

print(f"{'segment':<20} {'x':>7} {'y':>7} {'heading':>9}")
print("-" * 46)

for label, secs, fwd, strafe, turn, arm in SCRIPT:
    for _ in range(int(secs / model.opt.timestep)):
        bot.drive(fwd, strafe, turn)
        bot.hold_arm(arm)
        bot.level_wrist(0.0)   # claw level; 1.45 (claw-down) would clip the 1.7 limit
        bot.set_grip(0.0)
        bot.apply()
        mujoco.mj_step(model, data)

        if len(frames) < data.time * FPS:
            # Orbit slowly so the mecanum rollers are visible from several angles.
            cam.azimuth = 130 + 18 * np.sin(data.time * 0.35)
            cam.lookat[:] = data.body("chassis").xpos
            renderer.update_scene(data, cam)
            frames.append(renderer.render())

    x, y, h = bot.pose()
    print(f"{label:<20} {x:>7.2f} {y:>7.2f} {np.degrees(h):>8.1f}d")

imageio.mimsave("/home/austin/ftcsim/teleop_demo.mp4", frames, fps=FPS, quality=8)
print(f"\nwrote teleop_demo.mp4  ({len(frames)} frames, {len(frames)/FPS:.1f}s)")

for i, f in enumerate([0.18, 0.42, 0.72, 0.93]):
    imageio.imwrite(f"/home/austin/ftcsim/tele{i}.png", frames[int(len(frames) * f)])
