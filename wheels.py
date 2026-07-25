"""Mecanum wheel presets, and the MJCF generator that builds them.

Different mecanum wheels behave differently, and the difference is mostly in the
rollers: how many, how big, and how soft. Softer rollers (lower durometer) grip
harder but wear faster and deform under load; harder rollers slip more but roll
with less resistance. This module makes that a one-line choice so you can
actually test it rather than argue about it.

VERIFIED vs ESTIMATED
    Diameter, roller count and durometer for the goBILDA wheels are from the
    manufacturer's published specs. The friction coefficients are ESTIMATES --
    durometer is a hardness rating, not a friction coefficient, and there is no
    clean conversion between them. They are ordered correctly relative to each
    other (softer grips more), which is what matters for comparing designs, but
    do not treat an absolute number as truth. If you measure your real robot's
    acceleration, tune `roller_friction` until the sim matches, and then the
    numbers mean something.
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class MecanumPreset:
    key: str
    name: str
    diameter_mm: float
    n_rollers: int
    durometer: str
    roller_friction: float     # ESTIMATE -- see module docstring
    width_mm: float
    mass_kg: float
    verified: str = ""
    note: str = ""

    @property
    def radius(self) -> float:
        return self.diameter_mm / 2000.0        # mm diameter -> m radius

    @property
    def roller_radius(self) -> float:
        # Rollers scale with the wheel; ~12.5% of diameter is close to the real
        # goBILDA geometry and keeps the rim looking right at any size.
        return self.diameter_mm * 0.125 / 1000.0

    @property
    def half_width(self) -> float:
        return self.width_mm / 2000.0


PRESETS = {
    "gobilda_96": MecanumPreset(
        key="gobilda_96",
        name="goBILDA 96mm Mecanum (70A)",
        diameter_mm=96.0, n_rollers=10, durometer="70A",
        roller_friction=1.10, width_mm=38.0, mass_kg=0.35,
        verified="diameter, roller count, durometer from goBILDA specs",
        note="The default FTC wheel. Harder rollers: less grip, less rolling "
             "resistance, longer life.",
    ),
    "gobilda_104": MecanumPreset(
        key="gobilda_104",
        name="goBILDA 104mm GripForce Mecanum (40A)",
        diameter_mm=104.0, n_rollers=11, durometer="40A",
        roller_friction=1.50, width_mm=38.0, mass_kg=0.40,
        verified="diameter, roller count, durometer from goBILDA specs",
        note="Soft silicone rollers, noticeably more grip. Larger diameter also "
             "raises top speed for the same motor.",
    ),
    "soft_test": MecanumPreset(
        key="soft_test",
        name="Hypothetical very soft rollers",
        diameter_mm=96.0, n_rollers=10, durometer="~30A",
        roller_friction=1.90, width_mm=38.0, mass_kg=0.35,
        verified="NOT A REAL PRODUCT",
        note="Not a product you can buy -- an upper bound, to see what grip is "
             "worth before spending money on it.",
    ),
    "hard_test": MecanumPreset(
        key="hard_test",
        name="Hypothetical hard rollers",
        diameter_mm=96.0, n_rollers=10, durometer="~90A",
        roller_friction=0.75, width_mm=38.0, mass_kg=0.35,
        verified="NOT A REAL PRODUCT",
        note="Deliberately slippery, to see what worn or glazed rollers cost you.",
    ),
}

DEFAULT = "gobilda_96"


def get(key):
    if key not in PRESETS:
        raise KeyError(f"unknown wheel preset {key!r}; options: {', '.join(PRESETS)}")
    return PRESETS[key]


def describe():
    lines = [f"{'key':<14} {'wheel':<38} {'rollers':>8} {'duro':>6} {'friction':>9}"]
    lines.append("-" * 80)
    for p in PRESETS.values():
        lines.append(f"{p.key:<14} {p.name:<38} {p.n_rollers:>8} {p.durometer:>6} "
                     f"{p.roller_friction:>9.2f}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# MJCF generation
# ---------------------------------------------------------------------------

def rollers_xml(preset, hand, wheel_name):
    """One wheel's worth of free-spinning rollers, in the wheel's local frame.

    The wheel's axle is local +X. Each roller sits on the rim with its spin axis
    tilted 45 degrees between the axle and the direction of travel. That tilt is
    the entire trick behind mecanum: it converts part of the wheel's forward
    force into sideways force, and four wheels combine those into strafing.
    """
    mount_r = preset.radius - preset.roller_radius
    half_len = preset.half_width * 0.85
    roller_mass = preset.mass_kg * 0.30 / preset.n_rollers

    out = []
    for i in range(preset.n_rollers):
        th = 2 * np.pi * i / preset.n_rollers
        pos = np.array([0.0, mount_r * np.cos(th), mount_r * np.sin(th)])

        axle = np.array([1.0, 0.0, 0.0])
        circumferential = np.array([0.0, -np.sin(th), np.cos(th)])
        axis = (axle + hand * circumferential) / np.sqrt(2.0)

        a, b = pos - axis * half_len, pos + axis * half_len
        out.append(f"""
        <body name="roller_{wheel_name}_{i}">
          <joint name="rollerj_{wheel_name}_{i}" type="hinge"
                 pos="{pos[0]:.5f} {pos[1]:.5f} {pos[2]:.5f}"
                 axis="{axis[0]:.5f} {axis[1]:.5f} {axis[2]:.5f}"
                 damping="0.00005" limited="false"/>
          <geom name="rollerg_{wheel_name}_{i}" type="capsule"
                size="{preset.roller_radius:.5f}"
                fromto="{a[0]:.5f} {a[1]:.5f} {a[2]:.5f} {b[0]:.5f} {b[1]:.5f} {b[2]:.5f}"
                mass="{roller_mass:.5f}"
                friction="{preset.roller_friction} 0.005 0.0001"
                contype="2" conaffinity="1" rgba="0.85 0.85 0.88 1"/>
        </body>""")
    return "".join(out)


def wheel_xml(preset, name, x, y, z, hand):
    """A driven hub carrying free rollers.

    Rotated 90 degrees about Z so the hub's local +X is the robot's left/right
    axis, which lets the roller maths above assume the axle is +X every time.
    """
    hub_r = preset.radius - preset.roller_radius - 0.004
    hub_mass = preset.mass_kg * 0.70
    return f"""
      <body name="wheel_{name}" pos="{x:.5f} {y:.5f} {z:.5f}" euler="0 0 1.5708">
        <joint name="drive_{name}" type="hinge" axis="1 0 0" damping="0.002" limited="false"/>
        <geom name="hub_{name}" type="cylinder"
              size="{hub_r:.5f} {preset.half_width:.5f}" euler="0 1.5708 0"
              mass="{hub_mass:.5f}" contype="0" conaffinity="0" rgba="0.25 0.25 0.3 1"/>
        {rollers_xml(preset, hand, name)}
      </body>"""


if __name__ == "__main__":
    print(describe())
    print()
    for p in PRESETS.values():
        print(f"{p.key}: {p.note}")
        print(f"    verified: {p.verified or 'nothing -- estimates only'}")
