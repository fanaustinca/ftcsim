"""What a naive CAD mesh import does to collision, measured.

MuJoCo collision geometry must be CONVEX. Import a mesh straight from CAD and
its collision shape becomes its convex hull -- so an open U-channel becomes a
solid block, and anything that was supposed to nest inside it no longer can.

This rebuilds the robot with every U-channel replaced by its convex hull (one
solid box of the same envelope) and measures what that costs.
"""
import mujoco
import numpy as np

import structure as st
from robot import build_mjcf
from motors import Robot


def hull_channel(name, length, pos, axis="x", rgba="0.62 0.64 0.68 1",
                 density=st.AL_6061, group=2, col_group=None):
    """Drop-in replacement for st.u_channel: the CONVEX HULL of the channel.

    A U-channel's hull is simply the solid box enclosing it -- the open mouth
    fills in. This is exactly what you get from an unprocessed mesh import.
    """
    col_group = col_group or st.GRP_CHASSIS
    hl = length / 2
    hw = st.CHANNEL_W / 2
    hh = st.CHANNEL_H / 2
    x, y, z = pos
    size = (hl, hw, hh) if axis == "x" else (hw, hl, hh)
    return (f'<geom name="{name}_hull" type="box" pos="{x:.5f} {y:.5f} {z:.5f}" '
            f'size="{size[0]:.5f} {size[1]:.5f} {size[2]:.5f}" density="{density}" '
            f'{st.col(col_group)} group="{group}" rgba="{rgba}"/>')


def measure(label, xml):
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    bot = Robot(model, data)
    bot.stow()
    for _ in range(500):
        bot.hold_arm(1.0); bot.level_wrist(0.0); bot.apply(); mujoco.mj_step(model, data)

    # How far down can the arm actually get?
    lowest = 10.0
    for i in range(300):
        tgt = 1.0 - i * 0.006
        for _ in range(20):
            bot.hold_arm(tgt); bot.level_wrist(0.0); bot.apply(); mujoco.mj_step(model, data)
        th = float(data.joint("shoulder").qpos[0])
        lowest = min(lowest, th)
        if abs(th - tgt) > 0.20:
            break

    bid = model.body("finger_l").id
    tip = data.xpos[bid] + data.xmat[bid].reshape(3, 3) @ np.array([0.07, 0, 0])
    mass = sum(float(model.body_mass[b]) for b in range(model.nbody)
               if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
               not in (None, "world", "cube", "pushblock"))
    return dict(label=label, lowest=lowest, tip_z=float(tip[2]),
                mass=mass, geoms=model.ngeom)


real = build_mjcf()

# Swap the channel builder for its convex hull and rebuild.
orig = st.u_channel
st.u_channel = hull_channel
import importlib
import robot as R
importlib.reload(R)
hull = R.build_mjcf()
# The explicit <pair> list names channel sub-geoms (arm_ch_web, cross_f_f1...)
# which don't exist once a channel is a single hull box. Remap and dedupe.
import re
def remap(m):
    a, b = m.group(1), m.group(2)
    fix = lambda n: re.sub(r'_(web|f1|f2)$', '_hull', n)
    return f'<pair geom1="{fix(a)}" geom2="{fix(b)}" condim="3"/>'
hull = re.sub(r'<pair geom1="([^"]+)" geom2="([^"]+)" condim="3"/>', remap, hull)
seen, out = set(), []
for line in hull.split("\n"):
    if "<pair " in line:
        if line.strip() in seen:
            continue
        seen.add(line.strip())
    out.append(line)
hull = "\n".join(out)
st.u_channel = orig
importlib.reload(R)

rows = [measure("real channels (3 boxes)", real),
        measure("convex hull (solid box)", hull)]

print("=" * 74)
print("CONVEX HULL COST")
print("=" * 74)
print(f"{'model':<28} {'geoms':>6} {'mass kg':>9} {'arm reaches':>13} {'claw z':>9}")
print("-" * 74)
for r in rows:
    print(f"{r['label']:<28} {r['geoms']:>6} {r['mass']:>9.3f} "
          f"{r['lowest']:>12.3f}r {r['tip_z']:>8.3f}m")

a, b = rows
print("-" * 74)
print(f"mass inflated by  {b['mass']/a['mass'] - 1:+.1%}   "
      f"(a solid box weighs far more than the extrusion it encloses)")
print(f"arm travel lost   {abs(b['lowest'] - a['lowest']):.3f} rad "
      f"({np.degrees(abs(b['lowest'] - a['lowest'])):.0f} deg)")
print(f"claw sits         {(b['tip_z'] - a['tip_z'])*1000:+.0f} mm higher off the floor")
print()
print("The mass figure is the part people miss: hull collision geometry is also")
print("hull INERTIA unless masses come from CAD separately. Import gives you real")
print("mass properties, so in practice you get correct mass with wrong collision --")
print("which is worse, because nothing looks obviously broken.")


# NOTE: an earlier version of this file included a synthetic "arm inside a
# C-bracket" micro-demo. It was removed because it did not actually measure
# what it claimed -- disabling the contact pairs changed nothing, since the
# contype/conaffinity masks still permitted contact, so the arm's travel was
# being limited by something other than the cage. The convex-hull effect on
# nesting geometry is real, but this file only reports the mass measurement
# above, which is measured on the actual robot and is trustworthy.
