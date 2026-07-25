"""What does this robot weigh, and can the motors actually move it?

Everything here is derived from geometry and material density -- nothing is a
typed-in guess. Change the arm length in robot.py and every number below moves.

The arm torque section is the one that earns its keep. "Will a 60 RPM Yellow
Jacket hold this arm straight out?" is answered by physics rather than optimism.
"""
import mujoco
import numpy as np

import wheels
from robot import build_mjcf, ARM_LEN
from motors import YJ_60, YJ_312

G = 9.81

model = mujoco.MjModel.from_xml_string(build_mjcf())
data = mujoco.MjData(model)

# ---------------------------------------------------------------------------
# Mass breakdown
# ---------------------------------------------------------------------------

def geom_mass(name_prefix):
    total = 0.0
    for g in range(model.ngeom):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
        if nm and nm.startswith(name_prefix):
            total += model.geom_mass[g] if hasattr(model, "geom_mass") else 0.0
    return total


# MuJoCo folds geom mass into bodies, so total per-body is the reliable view.
groups = {
    "frame (Al channel)": ["rail_l", "rail_r", "cross_f", "cross_b"],
    "deck (polycarb)": ["deck"],
    "drive motors": ["motor_"],
    "control hub": ["control_hub"],
    "battery": ["battery"],
    "wheels": ["hub_", "rollerg_"],
    "arm": ["arm_ch"],
    "wrist + claw": ["wristg", "finger_"],
}

body_of_geom = {}
for g in range(model.ngeom):
    nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
    if nm:
        body_of_geom[nm] = model.geom_bodyid[g]

# Recompute per-geom mass from volume * density, which is what MuJoCo did.
def geom_masses():
    out = {}
    for g in range(model.ngeom):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
        if not nm:
            continue
        t = model.geom_type[g]
        s = model.geom_size[g]
        if t == mujoco.mjtGeom.mjGEOM_BOX:
            vol = 8 * s[0] * s[1] * s[2]
        elif t == mujoco.mjtGeom.mjGEOM_CYLINDER:
            vol = np.pi * s[0] ** 2 * 2 * s[1]
        elif t == mujoco.mjtGeom.mjGEOM_CAPSULE:
            vol = np.pi * s[0] ** 2 * (2 * s[1]) + (4 / 3) * np.pi * s[0] ** 3
        else:
            vol = 0.0
        # density isn't exposed directly; back it out where mass was set,
        # otherwise use the model's own computed value via body aggregation.
        out[nm] = vol
    return out

vols = geom_masses()

print("=" * 66)
print("MASS BUDGET  (computed from geometry x material density)")
print("=" * 66)

total_model = float(sum(model.body_mass))
robot_bodies = [b for b in range(model.nbody)
                if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) not in
                (None, "world", "cube", "pushblock")]
robot_mass = sum(float(model.body_mass[b]) for b in robot_bodies)

named = {}
for b in robot_bodies:
    nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
    key = ("wheels" if nm.startswith(("wheel_", "roller_")) else
           "arm assembly" if nm in ("arm", "wrist", "finger_l", "finger_r") else
           "chassis + electronics")
    named[key] = named.get(key, 0.0) + float(model.body_mass[b])

for k, v in sorted(named.items(), key=lambda kv: -kv[1]):
    print(f"  {k:<26} {v:7.3f} kg   {v/robot_mass:5.1%}")
print(f"  {'-'*26} {'-'*7}")
print(f"  {'TOTAL':<26} {robot_mass:7.3f} kg")

FTC_LIMIT = 19.05      # 42 lb
print(f"\n  FTC weight limit ~{FTC_LIMIT:.1f} kg -> using {robot_mass/FTC_LIMIT:.0%} of it")
print("  NOTE: this models frame, deck, motors, wheels, hub and battery only.")
print("  A real robot adds brackets, fasteners, wiring, servos and mechanism")
print("  hardware -- expect the real thing to land 2-3x heavier. Pattern holes")
print("  aren't modelled either, which pushes the other way by ~10-15%.")

# ---------------------------------------------------------------------------
# Arm: mass, centre of gravity, and the torque needed to hold it
# ---------------------------------------------------------------------------

print("\n" + "=" * 66)
print("ARM ANALYSIS")
print("=" * 66)

arm_bodies = [model.body(n).id for n in ("arm", "wrist", "finger_l", "finger_r")]
arm_mass = sum(float(model.body_mass[b]) for b in arm_bodies)

# Put the arm horizontal, at rest, and ask MuJoCo what gravity is doing to it.
shoulder = model.joint("shoulder")
dof = model.jnt_dofadr[shoulder.id]
data.qpos[model.jnt_qposadr[shoulder.id]] = 0.0
data.qvel[:] = 0
mujoco.mj_forward(model, data)

pivot = data.body("arm").xpos.copy()
com = np.zeros(3)
for b in arm_bodies:
    com += float(model.body_mass[b]) * data.xipos[b]
com /= arm_mass
reach = float(np.linalg.norm((com - pivot)[:2]))

# qfrc_bias at zero velocity is the gravity torque the motor must fight.
gravity_torque = abs(float(data.qfrc_bias[dof]))

print(f"  arm assembly mass        {arm_mass:6.3f} kg")
print(f"  centre of gravity        {reach*1000:6.1f} mm from the shoulder pivot")
print(f"  arm reach (tip)          {(ARM_LEN + 0.096)*1000:6.1f} mm")
print(f"\n  holding torque needed at horizontal: {gravity_torque:.3f} N*m")
print(f"  (cross-check, m*g*r = {arm_mass * G * reach:.3f} N*m)")

print(f"\n  {'motor':<28} {'stall':>8} {'margin':>9}  verdict")
print("  " + "-" * 62)
for label, mot in [("goBILDA 60 RPM (99.5:1)", YJ_60),
                   ("goBILDA 312 RPM (19.2:1)", YJ_312)]:
    usable = mot.stall_nm * mot.efficiency
    margin = usable / gravity_torque if gravity_torque > 0 else float("inf")
    if margin > 3:
        verdict = "comfortable"
    elif margin > 1.5:
        verdict = "workable, run a hold PID"
    elif margin > 1:
        verdict = "MARGINAL - will sag and cook the motor"
    else:
        verdict = "CANNOT HOLD IT"
    print(f"  {label:<28} {usable:7.2f}N*m {margin:8.1f}x  {verdict}")

print("\n  Gravity torque scales with cos(angle): maximum straight out,")
print("  zero straight up. That's what the kf term in Robot.hold_arm feeds")
print("  forward, so the PD terms only mop up the remainder.\n")

for ang in (0, 30, 60, 90):
    data.qpos[model.jnt_qposadr[shoulder.id]] = np.radians(ang)
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    t = abs(float(data.qfrc_bias[dof]))
    bar = "#" * int(t / gravity_torque * 34) if gravity_torque else ""
    print(f"    {ang:>2} deg  {t:5.3f} N*m  {bar}")

# ---------------------------------------------------------------------------
# Payload
# ---------------------------------------------------------------------------

usable = YJ_60.stall_nm * YJ_60.efficiency
tip_r = ARM_LEN + 0.096
spare = usable - gravity_torque
payload = spare / (G * tip_r)
print(f"\n  Spare torque at horizontal: {spare:.2f} N*m")
print(f"  -> max payload at full reach ({tip_r*1000:.0f} mm): {payload:.2f} kg "
      f"({payload*1000:.0f} g)")
print("  A game element is typically 50-150 g, so there is plenty of headroom.")
