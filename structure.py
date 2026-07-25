"""Structural members and materials, so mass is computed rather than guessed.

THE POINT
    Previously every mass in this project was a number typed in by hand:
    CHASSIS_MASS = 10.0, arm mass="1.1". Those were plausible guesses, and
    guesses are worthless for the questions this simulator exists to answer.
    "Can a 60 RPM motor hold this arm out horizontally?" is entirely a question
    about the arm's mass and where its centre of gravity sits.

    So instead of assigning mass, we build parts out of aluminium channel and
    polycarbonate plate at their real densities and let MuJoCo integrate the
    geometry. Change the arm length and the mass follows automatically -- which
    is what makes the answer trustworthy.

DIMENSIONS
    Modelled on goBILDA's 1120-series U-channel and 8 mm pattern hardware.
    Section sizes are close but not exact, and the pattern holes are not
    modelled (they'd remove maybe 10-15% of the mass). Treat computed masses as
    good to ~15%, which is still enormously better than a typed-in guess. If you
    weigh a real part, adjust the density to match.
"""

# Densities, kg/m^3. These are the real material values.
AL_6061 = 2700.0        # aluminium extrusion / channel / plate
POLYCARB = 1200.0       # the clear plate everyone builds mounts from
ABS = 1040.0            # printed parts, servo horns
STEEL = 7850.0          # shafts, fasteners
NEOPRENE = 1230.0       # roller compound

# goBILDA-ish U-channel section.
CHANNEL_W = 0.048       # outer width
CHANNEL_H = 0.024       # outer height of the walls
WALL = 0.003            # material thickness
PLATE_T = 0.0032        # 1/8" polycarbonate


def u_channel(name, length, pos, axis="x", rgba="0.62 0.64 0.68 1",
              density=AL_6061, group=2):
    """A U-shaped aluminium channel: a base web plus two upstanding flanges.

    Modelled as three thin boxes rather than one solid block. That matters for
    mass -- a solid box of the same envelope would weigh roughly four times as
    much as the real extrusion, which would wreck every torque calculation
    downstream.

    axis: 'x' runs the channel fore-aft, 'y' runs it side to side.
    """
    hl = length / 2
    hw = CHANNEL_W / 2
    hh = CHANNEL_H / 2
    t = WALL / 2
    x, y, z = pos

    if axis == "x":
        parts = [
            # web (the flat bottom of the U)
            (f"{name}_web", (x, y, z - hh + t), (hl, hw, t)),
            # two flanges (the upstanding sides)
            (f"{name}_f1", (x, y + hw - t, z), (hl, t, hh)),
            (f"{name}_f2", (x, y - hw + t, z), (hl, t, hh)),
        ]
    else:
        parts = [
            (f"{name}_web", (x, y, z - hh + t), (hw, hl, t)),
            (f"{name}_f1", (x + hw - t, y, z), (t, hl, hh)),
            (f"{name}_f2", (x - hw + t, y, z), (t, hl, hh)),
        ]

    out = []
    for gname, (px, py, pz), (sx, sy, sz) in parts:
        out.append(
            f'<geom name="{gname}" type="box" pos="{px:.5f} {py:.5f} {pz:.5f}" '
            f'size="{sx:.5f} {sy:.5f} {sz:.5f}" density="{density}" '
            f'contype="2" conaffinity="1" group="{group}" rgba="{rgba}"/>')
    return "\n        ".join(out)


def plate(name, sx, sy, pos, thickness=PLATE_T, density=POLYCARB,
          rgba="0.55 0.62 0.70 0.55", group=2):
    x, y, z = pos
    return (f'<geom name="{name}" type="box" pos="{x:.5f} {y:.5f} {z:.5f}" '
            f'size="{sx/2:.5f} {sy/2:.5f} {thickness/2:.5f}" density="{density}" '
            f'contype="2" conaffinity="1" group="{group}" rgba="{rgba}"/>')


def motor(name, pos, euler="0 0 0", rgba="0.20 0.20 0.23 1"):
    """A Yellow Jacket-ish gearmotor: 37 mm body, ~63 mm long.

    Density is tuned so the modelled cylinder lands near the real ~0.30 kg --
    a motor is mostly copper and steel, not aluminium, so using AL here would
    under-weigh it badly. Four of these sit low in the chassis and are a real
    part of the mass budget.
    """
    x, y, z = pos
    return (f'<geom name="{name}" type="cylinder" pos="{x:.5f} {y:.5f} {z:.5f}" '
            f'euler="{euler}" size="0.0185 0.0315" density="4400" '
            f'contype="0" conaffinity="0" group="2" rgba="{rgba}"/>')


def box_part(name, size, pos, density, rgba, contact=False):
    """Generic box: control hub, battery, etc."""
    x, y, z = pos
    sx, sy, sz = size
    ct = '2' if contact else '0'
    ca = '1' if contact else '0'
    return (f'<geom name="{name}" type="box" pos="{x:.5f} {y:.5f} {z:.5f}" '
            f'size="{sx/2:.5f} {sy/2:.5f} {sz/2:.5f}" density="{density}" '
            f'contype="{ct}" conaffinity="{ca}" group="2" rgba="{rgba}"/>')
