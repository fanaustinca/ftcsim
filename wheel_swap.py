"""Detect CAD wheels in an imported model and swap in simulated mecanum wheels.

WHY THIS EXISTS
    Wheels exported from CAD are the one part of an Onshape import that is
    actively wrong. A mecanum wheel in CAD is a rigid lump of geometry: its
    rollers are modelled as shapes, not as bodies that spin freely. Simulate
    that directly and you get a robot that cannot strafe at all -- the rollers
    are welded, so every wheel just grips forward like a traction wheel.

    So: find the CAD wheels, delete them, and graft on generated wheels whose
    rollers are real hinged bodies. Everything else from the import (chassis,
    arm, masses, inertia) is kept exactly as exported.

USAGE
    python wheel_swap.py imported.xml --preset gobilda_104 -o robot_fixed.xml
    python wheel_swap.py imported.xml --list          # just report what it finds
"""
import argparse
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

import wheels

# Names CAD tends to produce for wheels. Case-insensitive.
WHEEL_PATTERN = re.compile(
    r"wheel|mecanum|meccanum|roller|tire|tyre|omni|hub|rim", re.IGNORECASE)


def parse_pos(el, default=(0.0, 0.0, 0.0)):
    raw = el.get("pos")
    return np.array([float(v) for v in raw.split()]) if raw else np.array(default)


def find_wheels(root):
    """Return [(parent_element, body_element, name, position), ...].

    Matches on body name. A body whose name looks like a wheel is taken whole,
    including any child roller geometry, since we're replacing the entire
    assembly rather than editing it.
    """
    found = []
    for parent in root.iter():
        for body in list(parent):
            if body.tag != "body":
                continue
            name = body.get("name", "")
            if WHEEL_PATTERN.search(name):
                found.append((parent, body, name, parse_pos(body)))
    return found


def infer_corner(pos, all_pos):
    """Work out which corner a wheel is at, and therefore its roller handedness.

    Mecanum handedness alternates diagonally: front-left and back-right share
    one roller direction, front-right and back-left the other. We decide
    front/back and left/right by comparing against the centroid of all the
    wheels, so it works regardless of which way the CAD had the robot facing.
    """
    centre = np.mean(all_pos, axis=0)
    front = pos[0] >= centre[0]
    left = pos[1] >= centre[1]
    name = ("f" if front else "b") + ("l" if left else "r")
    hand = 1 if (front == left) else -1        # fl,br -> +1 ; fr,bl -> -1
    return name, hand


def swap(in_path, out_path, preset_key, dry_run=False):
    preset = wheels.get(preset_key)
    tree = ET.parse(in_path)
    root = tree.getroot()

    found = find_wheels(root)
    if not found:
        print("No wheel-like bodies found. Nothing to do.")
        print("The detector matches body names against: "
              f"{WHEEL_PATTERN.pattern}")
        print("If your CAD uses different names, rename the mates in Onshape or "
              "widen WHEEL_PATTERN.")
        return False

    positions = [p for _, _, _, p in found]
    print(f"Found {len(found)} wheel-like bodies in {in_path}:")
    assignments = []
    for (parent, body, name, pos) in found:
        corner, hand = infer_corner(pos, positions)
        ngeom = len(body.findall(".//geom"))
        njoint = len(body.findall(".//joint"))
        assignments.append((parent, body, name, pos, corner, hand))
        print(f"  {name:<28} pos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})  "
              f"{ngeom:>2} geoms {njoint:>2} joints  -> corner {corner}")

    if len(found) != 4:
        print(f"\nWARNING: expected 4 wheels, found {len(found)}. "
              "Corner assignment is probably wrong -- check the list above.")

    corners = [a[4] for a in assignments]
    if len(set(corners)) != len(corners):
        print(f"\nWARNING: duplicate corners {corners}. Two wheels resolved to "
              "the same position; the mecanum handedness will be wrong.")

    if dry_run:
        print("\n--list only, nothing written.")
        return True

    print(f"\nReplacing with: {preset.name}")
    print(f"  {preset.n_rollers} rollers, {preset.durometer}, "
          f"friction {preset.roller_friction}")

    new_xml = []
    for (parent, body, name, pos, corner, hand) in assignments:
        parent.remove(body)
        new_xml.append(wheels.wheel_xml(preset, corner, pos[0], pos[1], pos[2], hand))

    # Graft the generated wheels onto whichever body held the originals.
    host = assignments[0][0]
    fragment = ET.fromstring("<wrap>" + "".join(new_xml) + "</wrap>")
    for child in fragment:
        host.append(child)

    tree.write(out_path, encoding="unicode")
    print(f"\nWrote {out_path}")
    print("Joints are named drive_fl / drive_fr / drive_bl / drive_br, "
          "so motors.py works unchanged.")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="MJCF file exported from Onshape")
    ap.add_argument("-o", "--output", default=None)
    ap.add_argument("--preset", default=wheels.DEFAULT,
                    choices=list(wheels.PRESETS))
    ap.add_argument("--list", action="store_true",
                    help="report detected wheels without modifying anything")
    a = ap.parse_args()

    if not a.list and not a.output:
        ap.error("give -o OUTPUT, or use --list to preview")

    ok = swap(a.input, a.output, a.preset, dry_run=a.list)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
