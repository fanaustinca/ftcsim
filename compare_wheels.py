"""Compare wheel presets on the measurements that matter to a driver.

This is the sim earning its keep: "are the grippier rollers worth it?" is a
design question with a real answer, and running it beats arguing about it.
"""
import mujoco
import numpy as np

import wheels
from robot import build_mjcf
from motors import Robot


def measure(key):
    model = mujoco.MjModel.from_xml_string(build_mjcf(key))
    data = mujoco.MjData(model)
    bot = Robot(model, data)
    bot.stow()
    for _ in range(500):
        bot.hold_arm(1.2)
        bot.apply()
        mujoco.mj_step(model, data)

    def run(fwd, strafe, turn, secs):
        d0 = mujoco.MjData(model)
        d0.qpos[:] = data.qpos
        d0.qvel[:] = 0
        b = Robot(model, d0)
        x0, y0, h0 = b.pose()
        t_hit, top = None, 0.0
        n = int(secs / model.opt.timestep)
        for i in range(n):
            b.drive(fwd, strafe, turn)
            b.hold_arm(1.2)
            b.apply()
            mujoco.mj_step(model, d0)
            v = float(np.linalg.norm(d0.body("chassis").cvel[3:5]))
            top = max(top, v)
            if t_hit is None and v > 0.5:
                t_hit = i * model.opt.timestep
        x1, y1, h1 = b.pose()
        dh = np.degrees((h1 - h0 + np.pi) % (2 * np.pi) - np.pi)
        return np.hypot(x1 - x0, y1 - y0), top, t_hit, dh

    fwd_d, fwd_top, fwd_t, _ = run(1, 0, 0, 2.0)
    str_d, str_top, _, str_yaw = run(0, 1, 0, 2.0)
    _, _, _, spin = run(0, 0, 1, 2.0)
    return dict(fwd_d=fwd_d, fwd_top=fwd_top, fwd_t=fwd_t,
                str_d=str_d, str_top=str_top, str_yaw=abs(str_yaw), spin=abs(spin))


print("2 s at full command, from a standstill.\n")
print(f"{'preset':<14} {'fwd (m)':>8} {'top m/s':>8} {'0-0.5 m/s':>10} "
      f"{'strafe (m)':>11} {'strafe/fwd':>11} {'yaw err':>8} {'spin deg':>9}")
print("-" * 88)

rows = {}
for key in wheels.PRESETS:
    r = measure(key)
    rows[key] = r
    t = f"{r['fwd_t']:.2f}s" if r["fwd_t"] else "  --  "
    print(f"{key:<14} {r['fwd_d']:>8.2f} {r['fwd_top']:>8.2f} {t:>10} "
          f"{r['str_d']:>11.2f} {r['str_d']/r['fwd_d']:>11.1%} "
          f"{r['str_yaw']:>7.1f}d {r['spin']:>8.1f}d")

print("-" * 88)
base = rows["gobilda_96"]
print("\nrelative to the stock goBILDA 96mm:")
for key, r in rows.items():
    if key == "gobilda_96":
        continue
    print(f"  {key:<14} forward {r['fwd_d']/base['fwd_d'] - 1:+6.1%}   "
          f"strafe {r['str_d']/base['str_d'] - 1:+6.1%}   "
          f"spin {r['spin']/base['spin'] - 1:+6.1%}")

# ---------------------------------------------------------------------------
# Pushing match: traction-limited, so this is where grip actually shows up.
# ---------------------------------------------------------------------------

def push_test(key, secs=2.5):
    """Drive into the field wall and measure how hard the robot can shove.

    Peak push force is min(what the motors can deliver, what the tyres can grip)
    = min(4*stall/radius, mu*m*g). Below about mu=1.5 the grip is the binding
    constraint, so this is the test where roller compound actually matters --
    unlike free driving, where the motor curve sets the speed and the wheels
    never slip.
    """
    model = mujoco.MjModel.from_xml_string(build_mjcf(key))
    data = mujoco.MjData(model)
    bot = Robot(model, data)
    bot.stow()
    data.qpos[0] = 1.50                      # start just short of the +X wall
    mujoco.mj_forward(model, data)
    for _ in range(500):
        bot.hold_arm(1.2); bot.apply(); mujoco.mj_step(model, data)

    wall_geoms = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"wall{i}")
                  for i in range(4)}
    # Average over the SETTLED portion. Taking the peak instead measures the
    # impact transient of hitting the wall, which reads far above what the
    # motors can actually deliver and is meaningless as "pushing power".
    force, samples = np.zeros(6), []
    n = int(secs / model.opt.timestep)
    for i in range(n):
        bot.drive(1, 0, 0); bot.hold_arm(1.2); bot.apply()
        mujoco.mj_step(model, data)
        if i > n * 0.7:                       # last 30%: pressed and steady
            total = 0.0
            for c in range(data.ncon):
                con = data.contact[c]
                if con.geom1 in wall_geoms or con.geom2 in wall_geoms:
                    mujoco.mj_contactForce(model, data, c, force)
                    total += abs(float(force[0]))
            samples.append(total)
    return float(np.mean(samples)) if samples else 0.0


print("\n\nPUSHING POWER: full throttle into the wall")
print(f"{'preset':<14} {'friction':>9} {'push force':>12} {'traction cap':>13} {'motor cap':>11}")
print("-" * 64)
push = {}
MOTOR_CAP = 4 * 2.4 * 0.85 / wheels.get("gobilda_96").radius
for key in wheels.PRESETS:
    f = push_test(key)
    push[key] = f
    mu = wheels.get(key).roller_friction
    cap = mu * 12.9 * 9.81
    print(f"{key:<14} {mu:>9.2f} {f:>11.1f}N {cap:>12.0f}N {MOTOR_CAP:>10.0f}N")
print("-" * 64)
b = push["gobilda_96"]
for key, d in push.items():
    if key != "gobilda_96":
        print(f"  {key:<14} {d/b - 1:+6.1%} vs stock 96mm")

print("\nStrafing is always slower than driving -- mecanum wastes some of every")
print("wheel's force sideways into the roller axis. The ratio above is how much.")
