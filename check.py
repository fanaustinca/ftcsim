"""Sanity check: does the model load, settle, and look right?

Renders a few frames to PNG so we can eyeball it without needing a GUI.
"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image

from robot import build_mjcf

model = mujoco.MjModel.from_xml_string(build_mjcf())
data = mujoco.MjData(model)

print(f"bodies={model.nbody}  joints={model.njnt}  geoms={model.ngeom}  actuators={model.nu}")
print(f"total mass = {sum(model.body_mass):.2f} kg")

# Let it fall and settle.
for _ in range(1500):
    mujoco.mj_step(model, data)

chassis = model.body("chassis").id
print(f"settled chassis height = {data.xpos[chassis][2]:.4f} m")
print(f"settled tilt (should be ~0) = {data.xpos[chassis][:2]}")

renderer = mujoco.Renderer(model, height=720, width=1280)
cam = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam)

for i, (az, el, dist) in enumerate([(140, -22, 1.6), (90, -55, 2.2)]):
    cam.azimuth, cam.elevation, cam.distance = az, el, dist
    cam.lookat[:] = data.xpos[chassis]
    renderer.update_scene(data, cam)
    Image.fromarray(renderer.render()).save(f"/home/austin/ftcsim/view{i}.png")

print("wrote view0.png view1.png")
