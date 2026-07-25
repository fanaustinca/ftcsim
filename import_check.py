"""Health-check a robot exported from Onshape, before you try to drive it.

    python import_check.py onshape_import/robot.xml

Every failure mode in docs/CAD_PREP.md is silent -- a robot with no materials
loads fine and weighs nothing, an unmated part loads fine and floats away, a
model with no joints loads fine and is one rigid lump. Nothing errors. So this
looks for each of them and says which one you have.
"""
import argparse
import sys

import mujoco
import numpy as np

OK, WARN, BAD = "ok  ", "WARN", "FAIL"


def check(path):
    try:
        model = mujoco.MjModel.from_xml_path(path)
    except Exception as e:
        print(f"{BAD}  model won't load at all:\n      {e}")
        return 1

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    issues = []

    def line(status, text, detail=""):
        print(f"  {status}  {text}")
        if detail:
            for d in str(detail).split("\n"):
                print(f"        {d}")
        if status == BAD:
            issues.append(text)

    name = lambda kind, i: mujoco.mj_id2name(model, kind, i) or f"<{i}>"
    bodies = [name(mujoco.mjtObj.mjOBJ_BODY, b) for b in range(model.nbody)]
    joints = [(name(mujoco.mjtObj.mjOBJ_JOINT, j), model.jnt_type[j])
              for j in range(model.njnt)]

    print(f"\n{'='*68}\nIMPORT CHECK: {path}\n{'='*68}\n")
    print(f"  {model.nbody-1} bodies, {model.njnt} joints, {model.ngeom} geoms, "
          f"{model.nu} actuators\n")

    # --- one rigid lump? -------------------------------------------------
    hinges = [n for n, t in joints if t in (mujoco.mjtJoint.mjJNT_HINGE,
                                            mujoco.mjtJoint.mjJNT_SLIDE)]
    if not hinges:
        line(BAD, "No hinge or slide joints at all -- the robot is one rigid lump.",
             "Your moving mates aren't Revolute/Slider, or aren't named dof_*.")
    else:
        line(OK, f"{len(hinges)} movable joint(s): {', '.join(hinges[:8])}"
                 + (" ..." if len(hinges) > 8 else ""))

    # --- floating / disconnected bodies ----------------------------------
    # Free bodies: one is the robot. Others may be legitimate game elements, so
    # rank by how much of the model hangs off each and report rather than fail.
    free_bodies = []
    for j in range(model.njnt):
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
            continue
        b = int(model.jnt_bodyid[j])
        subtree = sum(1 for k in range(model.nbody)
                      if k == b or model.body_rootid[k] == model.body_rootid[b])
        free_bodies.append((bodies[b], float(model.body_subtreemass[b]), subtree))
    free_bodies.sort(key=lambda t: -t[1])

    if not free_bodies:
        line(WARN, "No free joint -- the robot is welded to the world.",
             "Fine for testing a mechanism, wrong for a drivebase.")
    elif len(free_bodies) == 1:
        line(OK, f"One free body: {free_bodies[0][0]} ({free_bodies[0][1]:.2f} kg).")
    else:
        main_b, main_m, _ = free_bodies[0]
        others = free_bodies[1:]
        detail = "\n".join(f"{n}  {m:.3f} kg" for n, m, _ in others)
        if all(m < main_m * 0.1 for _, m, _ in others):
            line(OK, f"Robot is {main_b} ({main_m:.2f} kg); "
                     f"{len(others)} light free body(s) look like game elements.",
                 detail)
        else:
            line(BAD, f"{len(free_bodies)} free bodies of comparable mass -- "
                      f"the robot is in pieces.",
                 "Unmated parts, or the export reported more than 1 root node.\n" + detail)

    # --- masses ----------------------------------------------------------
    total = float(sum(model.body_mass))
    massless = [bodies[b] for b in range(1, model.nbody)
                if model.body_mass[b] < 1e-6]
    if total < 0.5:
        line(BAD, f"Total mass is {total:.4f} kg -- essentially nothing.",
             "Parts are missing materials in Onshape.")
    elif massless:
        line(WARN, f"{len(massless)} body(s) weigh ~0: "
                   f"{', '.join(massless[:5])}{' ...' if len(massless) > 5 else ''}",
             "Those parts probably have no material assigned.")
    else:
        line(OK, f"Total mass {total:.3f} kg, every body has mass.")

    if total > 0.5:
        fine = 19.05
        pct = total / fine
        if pct > 1.0:
            line(WARN, f"That's {pct:.0%} of the ~19 kg FTC limit -- over weight.")
        elif pct < 0.15:
            line(WARN, f"That's only {pct:.0%} of the FTC limit; light for a real robot.")

    # --- inertia sanity --------------------------------------------------
    bad_inertia = [bodies[b] for b in range(1, model.nbody)
                   if model.body_mass[b] > 1e-6 and model.body_inertia[b].min() <= 0]
    if bad_inertia:
        line(BAD, f"{len(bad_inertia)} body(s) have non-positive inertia.",
             "Usually degenerate geometry; the sim will be unstable.")
    else:
        line(OK, "Inertia tensors look sane.")

    # --- actuators -------------------------------------------------------
    if model.nu == 0:
        line(WARN, "No actuators. The exporter makes joints, not motors.",
             "You'll add <motor>/<position> entries, then configure.py.")
    else:
        line(OK, f"{model.nu} actuator(s) present.")

    # --- wheels ----------------------------------------------------------
    import re
    geoms = [name(mujoco.mjtObj.mjOBJ_GEOM, g) for g in range(model.ngeom)]
    wheelish = [b for b in bodies if re.search(r"wheel|mecanum|tire|omni", b, re.I)]
    roller_joints = [n for n, t in joints if re.search(r"roller", n, re.I)]
    # Look for rollers in GEOM names too: a CAD export models them as geometry
    # inside a rigid wheel body, so there is no roller *body* to find.
    roller_geoms = [g for g in geoms if re.search(r"roller", g, re.I)]
    mecanum_named = [b for b in bodies if re.search(r"mecanum", b, re.I)]

    if roller_joints:
        line(OK, f"{len(roller_joints)} roller joints -- wheels can spin freely.")
    elif roller_geoms or mecanum_named:
        line(BAD, "Mecanum wheels found, but the rollers have NO joints.",
             f"{len(roller_geoms)} roller geoms / {len(mecanum_named)} mecanum bodies,\n"
             "all rigid. This robot CANNOT STRAFE. Fix with:\n"
             "  python wheel_swap.py <file> --preset gobilda_96 -o fixed.xml")
    elif wheelish:
        line(WARN, f"{len(wheelish)} wheel body(s), no rollers detected.",
             "Fine for a tank drive. If it's meant to be mecanum, the rollers\n"
             "didn't export as separate parts -- run wheel_swap.py.")
    else:
        line(WARN, "No wheel-like body names found.",
             "If this is a drivebase, wheel_swap.py won't recognise the names.\n"
             "Check with:  python wheel_swap.py <file> --list")

    # --- interpenetration at rest ---------------------------------------
    deep = [(name(mujoco.mjtObj.mjOBJ_GEOM, data.contact.geom1[c]),
             name(mujoco.mjtObj.mjOBJ_GEOM, data.contact.geom2[c]),
             float(data.contact.dist[c]))
            for c in range(data.ncon) if data.contact.dist[c] < -0.002]
    if deep:
        line(BAD, f"{len(deep)} geom pair(s) start deeply overlapped.",
             "The solver will fight to separate them and the robot may explode.\n"
             "Usually a convex hull filling a gap. Worst:\n"
             + "\n".join(f"  {a} <-> {b}  {d*1000:.1f} mm" for a, b, d in
                         sorted(deep, key=lambda x: x[2])[:3]))
    else:
        line(OK, "Nothing starts interpenetrating.")

    # --- does it just sit there? ----------------------------------------
    for _ in range(1000):
        mujoco.mj_step(model, data)
    if not np.all(np.isfinite(data.qpos)):
        line(BAD, "Simulation blew up within 2 seconds (NaN positions).")
    else:
        drift = float(np.linalg.norm(data.qvel[:3])) if model.nv >= 3 else 0.0
        if drift > 1.0:
            line(WARN, f"Robot is still moving at {drift:.2f} m/s after settling.",
                 "It may be falling, sliding, or being pushed apart.")
        else:
            line(OK, "Settles quietly under gravity.")

    print()
    if issues:
        print(f"  {len(issues)} blocking problem(s). See docs/CAD_PREP.md.")
        return 1
    print("  No blocking problems. Next:")
    print("    python wheel_swap.py <file> --preset gobilda_96 -o fixed.xml")
    print("    python configure.py fixed.xml --auto -o my_robot.json")
    print("    python teleop.py --config my_robot.json")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("model")
    sys.exit(check(ap.parse_args().model))
