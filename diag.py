"""Per-wheel diagnostic: drive one wheel at a time, see which way the robot goes.

A single mecanum wheel should push the robot along a 45-degree diagonal. Which
diagonal tells us whether the roller handedness matches the drive mixing.
"""
import mujoco
import numpy as np

from robot import build_mjcf

model = mujoco.MjModel.from_xml_string(build_mjcf())
names = ["fl", "fr", "bl", "br"]

print(f"{'wheel':<6} {'dX':>8} {'dY':>8} {'heading':>9}  {'wheel rad/s':>12} {'contacts':>9}")
print("-" * 60)

for i, nm in enumerate(names):
    data = mujoco.MjData(model)
    for _ in range(400):
        mujoco.mj_step(model, data)

    start = data.body("chassis").xpos.copy()
    data.ctrl[i] = 3.0
    for _ in range(750):
        mujoco.mj_step(model, data)

    d = data.body("chassis").xpos - start
    spin = data.joint(f"drive_{nm}").qvel[0]
    ang = np.degrees(np.arctan2(d[1], d[0]))
    print(f"{nm:<6} {d[0]:>8.3f} {d[1]:>8.3f} {ang:>8.1f}d {spin:>12.2f} {data.ncon:>9}")

# How many rollers are actually touching the ground at rest?
data = mujoco.MjData(model)
for _ in range(400):
    mujoco.mj_step(model, data)
print(f"\ncontacts at rest: {data.ncon}")
for c in range(data.ncon):
    g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact.geom1[c])
    g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact.geom2[c])
    print(f"  {g1} <-> {g2}   depth={data.contact.dist[c]:.5f}")
