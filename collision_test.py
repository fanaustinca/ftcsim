"""Verify every collision pair does what the group table claims.

Bitmask arithmetic is easy to get subtly wrong and the failure is silent -- the
arm simply passes through the robot and nothing complains. So assert it.
"""
import mujoco
import numpy as np

import structure as st
from robot import build_mjcf
from motors import Robot

model = mujoco.MjModel.from_xml_string(build_mjcf())

GROUPS = {
    "world": ("floor", "wall0"),
    "chassis": ("rail_l_web", "deck", "control_hub", "battery", "motor_fl"),
    "arm": ("arm_ch_web", "wristg", "finger_l_plate"),
    # hub_* and grip_servo are deliberately visual-only (the rollers make all
    # the wheel contact), so they're excluded rather than treated as failures.
    "wheel": ("rollerg_fl_0",),
    "game": ("cubeg",),
}

EXPECTED = {
    ("world", "chassis"): True,  ("world", "arm"): True,
    ("world", "wheel"): True,    ("world", "game"): True,
    ("chassis", "arm"): True,    ("chassis", "wheel"): False,
    ("chassis", "game"): True,   ("arm", "wheel"): False,
    ("arm", "game"): True,       ("wheel", "game"): True,
    ("chassis", "chassis"): False, ("arm", "arm"): False,
    ("wheel", "wheel"): False,
}


def collides(g1, g2):
    a, b = model.geom(g1).id, model.geom(g2).id
    ct1, ca1 = int(model.geom_contype[a]), int(model.geom_conaffinity[a])
    ct2, ca2 = int(model.geom_contype[b]), int(model.geom_conaffinity[b])
    return bool((ct1 & ca2) or (ct2 & ca1))


print("collision matrix")
print("-" * 52)
fails = 0
for (ga, gb), want in EXPECTED.items():
    names_a, names_b = GROUPS[ga], GROUPS[gb]
    results = set()
    for na in names_a:
        for nb in names_b:
            if na == nb:
                continue
            results.add(collides(na, nb))
    if not results:
        continue          # a self-pair with only one sample geom; nothing to check
    got = results.pop() if len(results) == 1 else f"MIXED {results}"
    ok = got is want
    fails += not ok
    print(f"  {ga:<8} <-> {gb:<8} want {str(want):<5} got {str(got):<5} "
          f"{'ok' if ok else '<-- WRONG'}")

# Geoms that collide with nothing at all are almost always a mistake.
print("\nnon-colliding geoms (contype=0 and conaffinity=0):")
dead = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
        for g in range(model.ngeom)
        if model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0]
print("  " + (", ".join(dead) if dead else "none"))

# --- behavioural check: drive the arm down into the deck ---------------
print("\nbehavioural: command the arm below the deck and see if it stops")
data = mujoco.MjData(model)
bot = Robot(model, data)
bot.stow()
for _ in range(400):
    bot.hold_arm(1.0); bot.level_wrist(0.0); bot.apply(); mujoco.mj_step(model, data)
for _ in range(1500):
    bot.set_arm(-1.0)               # full power downward, no limit respected
    bot.level_wrist(0.0); bot.apply(); mujoco.mj_step(model, data)

theta = float(data.joint("shoulder").qpos[0])
hits = [(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact.geom1[c]),
         mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact.geom2[c]))
        for c in range(data.ncon)]
arm_chassis = [h for h in hits
               if any("arm_ch" in g or "finger" in g or "wrist" in g for g in h)
               and any(k in g for g in h
                       for k in ("deck", "rail", "cross", "hub", "battery", "motor"))]
print(f"  shoulder settled at {theta:+.3f} rad (joint lower limit is -0.45)")
print(f"  arm<->chassis contacts: {len(arm_chassis)}")
for a, b in arm_chassis[:4]:
    print(f"    {a} <-> {b}")

print("\nRESULT:", "ALL PASS" if fails == 0 and arm_chassis else
      f"{fails} matrix failures" + ("" if arm_chassis else ", arm still passes through"))
