"""Scripted autonomous: drive to the cube, grab it, lift, back away.

Verifies the whole stack works together and renders a video.
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
cam.azimuth, cam.elevation, cam.distance = 135, -20, 1.7

CUBE_START_Z = 0.038
FPS = 50

# Found by tune_grab.py rather than by hand. stop_x/shoulder chosen from a cell
# whose neighbours also work, so the routine tolerates a bit of drive error.
STOP_X = 0.69
GRAB_SHOULDER = -0.40
WRIST_DOWN = 1.45      # positive rotates the claw to point downward
frames = []
log = []


def phase(name, seconds, drive=(0, 0, 0), arm=None, grip=None, goto_x=None):
    """Run one step of the routine, recording frames as we go.

    goto_x closes the loop on position with a P controller instead of driving
    open-loop for a fixed time -- the same reason real autos use encoders rather
    than sleep(). Open-loop timing overshot the cube by 20 cm.
    """
    for i in range(int(seconds / model.opt.timestep)):
        if goto_x is not None:
            err = goto_x - bot.pose()[0]
            bot.drive(float(np.clip(2.5 * err, -0.6, 0.6)), 0, 0)
        else:
            bot.drive(*drive)
        if arm is not None:
            bot.hold_arm(arm)
            bot.level_wrist(WRIST_DOWN)
        if grip is not None:
            bot.set_grip(grip)
        bot.apply()
        mujoco.mj_step(model, data)

        if len(frames) < (data.time * FPS):
            cam.lookat[:] = data.body("chassis").xpos
            renderer.update_scene(data, cam)
            frames.append(renderer.render())

    cube_z = float(data.body("cube").xpos[2])
    x, y, h = bot.pose()
    log.append((name, x, y, cube_z))
    print(f"{name:<22} robot=({x:+.2f},{y:+.2f})  cube_height={cube_z:.3f}")


print(f"{'phase':<22} {'robot pose':<22} cube")
print("-" * 60)

phase("settle",        0.8, arm=1.2, grip=0.0)
phase("drive to cube", 2.2, goto_x=STOP_X, arm=1.2, grip=0.0)
phase("lower over cube", 1.8, goto_x=STOP_X, arm=GRAB_SHOULDER, grip=0.0)
phase("close claw",    1.4, goto_x=STOP_X, arm=GRAB_SHOULDER, grip=1.0)
phase("lift",          1.8, goto_x=STOP_X, arm=1.0, grip=1.0)
phase("back away",     1.4, drive=(-0.5, 0, 0), arm=1.0, grip=1.0)
phase("hold",          0.8, arm=1.0, grip=1.0)

# "cube is higher than it started" is NOT a pickup test -- the arm can shove the
# cube onto the chassis deck and score a false pass. The real question is whether
# the cube is still in the claw, so measure the distance from cube to wrist.
cube = data.body("cube").xpos
wrist = data.body("wrist").xpos
held_dist = float(np.linalg.norm(cube - wrist))
off_floor = float(cube[2]) - CUBE_START_Z

print("-" * 60)
print(f"cube z      = {cube[2]:.3f}  (started {CUBE_START_Z:.3f})")
print(f"cube->wrist = {held_dist:.3f} m")
if held_dist < 0.12 and off_floor > 0.05:
    print("PICKUP SUCCEEDED - cube is in the claw")
elif off_floor > 0.05:
    print("FALSE PASS - cube is raised but NOT in the claw "
          "(probably resting on the chassis)")
else:
    print("PICKUP FAILED - cube never left the floor")

imageio.mimsave("/home/austin/ftcsim/demo.mp4", frames, fps=FPS, quality=8)
print(f"wrote demo.mp4 ({len(frames)} frames)")

for i, idx in enumerate([int(len(frames) * f) for f in (0.35, 0.60, 0.85)]):
    imageio.imwrite(f"/home/austin/ftcsim/demo{i}.png", frames[min(idx, len(frames) - 1)])
