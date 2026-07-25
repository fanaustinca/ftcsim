"""Is one collision shape per LINK better than one per part?

The idea: parts inside a link are welded together anyway, so give the whole
link a single collision shape instead of one per part. onshape-to-robot
supports it (merge_stls). Fewer shapes, faster sim.

The catch is what "single shape" means. MuJoCo needs convex geometry, so a
merged link becomes the convex hull of ALL its parts -- and a link's hull is
much bigger than the union of its parts' hulls. Every gap between parts fills
in solid.

This measures it on the real chassis.
"""
import re

import mujoco
import numpy as np

import structure as st
from robot import build_mjcf
from motors import Robot

real = build_mjcf()

# Convex hull of the whole chassis link: one box spanning everything on it,
# frame + deck + electronics + arm tower.
HULL_BOX = ('<geom name="chassis_hull" type="box" pos="0 0 0.0405" '
            'size="0.18 0.18 0.1025" density="700" '
            f'{st.col(st.GRP_CHASSIS)} group="2" rgba="0.45 0.48 0.55 0.75"/>')

merged = real
# strip every chassis-owned geom, keep wheels, arm, world, game piece
for name in ["rail_l", "rail_r", "cross_f", "cross_b", "deck", "riser_l",
             "riser_r", "control_hub", "battery", "motor_"]:
    merged = re.sub(rf'<geom name="{name}[^"]*"[^>]*/>', '', merged)
merged = merged.replace('<site name="imu"', HULL_BOX + '\n      <site name="imu"')
# repoint the explicit arm<->chassis pairs at the single hull
merged = re.sub(r'<pair geom1="([^"]+)" geom2="[^"]+" condim="3"/>',
                lambda m: f'<pair geom1="{m.group(1)}" geom2="chassis_hull" condim="3"/>',
                merged)
seen, out = set(), []
for line in merged.split("\n"):
    if "<pair " in line:
        if line.strip() in seen:
            continue
        seen.add(line.strip())
    out.append(line)
merged = "\n".join(out)


def arm_range(xml, label):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    bot = Robot(m, d)
    bot.stow()
    for _ in range(500):
        bot.hold_arm(1.0); bot.level_wrist(0.0); bot.apply(); mujoco.mj_step(m, d)

    lowest = 10.0
    for i in range(300):
        tgt = 1.0 - i * 0.006
        for _ in range(20):
            bot.hold_arm(tgt); bot.level_wrist(0.0); bot.apply(); mujoco.mj_step(m, d)
        th = float(d.joint("shoulder").qpos[0])
        lowest = min(lowest, th)
        if abs(th - tgt) > 0.20:
            break
    bid = m.body("finger_l").id
    tip = d.xpos[bid] + d.xmat[bid].reshape(3, 3) @ np.array([0.07, 0, 0])
    return label, m.ngeom, lowest, float(tip[2])


print("=" * 76)
print("ONE COLLISION SHAPE PER PART  vs  ONE PER LINK")
print("=" * 76)
print(f"{'model':<34} {'geoms':>6} {'arm reaches':>13} {'claw height':>13}")
print("-" * 76)
rows = [arm_range(real, "per part (current)"),
        arm_range(merged, "per link (merged hull)")]
for label, ng, lo, tz in rows:
    print(f"{label:<34} {ng:>6} {lo:>12.3f}r {tz:>12.3f}m")

print("-" * 76)
a, b = rows
print(f"geoms saved:  {a[1] - b[1]}  ({(a[1]-b[1])/a[1]:.0%} fewer)")
print(f"arm travel:   {np.degrees(abs(b[2]-a[2])):.0f} deg lost")
print(f"claw floor gap: {a[3]*1000:.0f} mm -> {b[3]*1000:.0f} mm")
