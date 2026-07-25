# Preparing your Onshape CAD so the import doesn't glitch out

Almost all the effort of importing a robot is spent in Onshape, not here. The
exporter is deterministic — feed it a clean assembly and it just works; feed it
a typical one and you get a robot that is one welded lump, or explodes on the
first frame, or has an arm that can't move.

Read this before you start mating. Fixing mates afterwards means redoing them.

---

## How the exporter decides what is attached to what

It never looks at position or proximity. It builds a graph — parts are nodes,
mates are edges — and sorts the edges into two kinds:

| mate name | what happens |
|---|---|
| `dof_something` | becomes a **joint**; splits the model into parent and child links |
| `fix_something` | **welds**; the parts become one rigid link |
| anything else | also welds |
| `closing_something` | closes a kinematic **loop** (four-bars, supported shafts) |
| `frame_something` | a named reference frame — useful for sensor mounts |
| `link_something` | names the resulting link |

A link is *everything reachable without crossing a `dof_` mate*. So a plate
belongs to the arm because it is fastened — directly, or through a chain of
other parts — to something on the far side of `dof_shoulder`.

**This is why unmated parts break things.** A part you positioned by dragging,
with no mate at all, becomes its own floating link. Nothing errors. You just get
a stray body in your sim.

---

## The checklist

### 1. Every moving joint is a real Revolute or Slider mate

Not Fastened. Not coincident-plus-tangent. Not "I dragged it until it stopped."

Rename it with a `dof_` prefix:

- `dof_shoulder`
- `dof_drive_fl`, `dof_drive_fr`, `dof_drive_bl`, `dof_drive_br`
- `dof_wrist`, `dof_grip_l`, `dof_grip_r`

Use exactly those drive names and `motors.py` works with no edits. Anything
else and you're renaming joints after export.

### 2. Every part has a material

Onshape → right-click part → **Assign material**. Aluminium 6061 for extrusion
and plate, ABS or PLA for printed parts, and something dense for motors and
batteries.

Mass and inertia come from here. A part with no material exports with junk mass,
**and nothing warns you**. Since the entire value of importing is getting real
mass properties, skipping this throws away the reason you imported.

### 3. Everything is connected

On export the tool prints:

```
* Found 1 root nodes:
  - chassis
```

**If that number is not 1, stop.** More than one root means your assembly is
actually several disconnected assemblies, and the extra ones will be floating
bodies. Find the unmated parts and fasten them.

### 4. Loops are marked

If a part reaches the chassis through *two* paths — a four-bar linkage, a shaft
held at both ends, a strut bracing an arm — the graph has a cycle and there is
no tree. Mark one mate in the loop `closing_` so the exporter knows where to
cut.

Unmarked loops are the most confusing failure, because the export "succeeds" and
the sim behaves nonsensically.

### 5. Sanity-check before you export

- Drag each joint through its full range in Onshape. Anything that collides
  there will collide in sim.
- Delete decorative parts you don't need. Every part is mesh triangles, and
  triangles cost frames.
- Check the mass in Onshape's measure tool. If your robot reads 400 g, some
  part is missing a material.

---

## Exporting

1. Get free API keys at <https://dev-portal.onshape.com/keys>. A personal
   Onshape account is enough — no paid plan needed.

   ```bash
   export ONSHAPE_ACCESS_KEY=your_access_key
   export ONSHAPE_SECRET_KEY=your_secret_key
   ```

   Put those in `~/.bashrc` so you don't redo it each session.

2. Paste your assembly URL into `onshape_import/config.json`.

3. Export:

   ```bash
   ./.venv/bin/onshape-to-robot onshape_import
   ```

---

## Collision: the part that will actually bite you

**MuJoCo collision geometry must be convex.** A mesh that isn't gets replaced by
its convex hull. An open U-channel becomes a solid block. A C-bracket's mouth
fills in. A claw becomes a solid wedge that cannot grip.

Measured on this repo's robot: replacing every U-channel with its convex hull
inflates mass by **63.7%**.

### Don't merge a whole link into one shape

Tempting, and `merge_stls` will do it: parts inside a link are welded anyway, so
why not give the link one collision shape? Because **a link's convex hull
swallows its own working volume**.

Measured here, collapsing the chassis into a single hull:

| | collision geoms | arm reaches |
|---|---|---|
| per part | 80 | −0.420 rad |
| per link (merged hull) | 60 | **+1.564 rad** |

25% fewer shapes, and **114° of arm travel gone** — the hull fills the space
above the deck where the arm swings, so the arm cannot come down at all.

The grouping instinct is fine. Hulling is what breaks it. If you want fewer
shapes, use convex *decomposition*, not a hull.

### What to actually do

In `onshape_import/config.json`:

```json
{
  "convex_decomposition": true,
  "simplify_stls": true,
  "max_stl_size": 3
}
```

`convex_decomposition` splits concave parts into convex pieces, so shapes
survive. `simplify_stls` caps mesh size — CAD meshes are far denser than physics
needs.

For anything that really matters — the chassis outline, the claw fingers, an
intake mouth — the professional answer is **meshes for looks, primitives for
contact**: set `no_collision_meshes` and place boxes and cylinders by hand.
Every serious robot model is built this way, and it is why `robot.py`'s
hand-built channels actually have *better* collision fidelity than a raw import
would.

---

## After exporting

Two things the export cannot give you:

### Mecanum wheels must be swapped

CAD rollers are rigid geometry, not bodies that spin. A raw import **cannot
strafe at all** — measured, 0.014 m of sideways travel versus 2.36 m after the
swap.

```bash
./.venv/bin/python wheel_swap.py onshape_import/robot.xml --list
./.venv/bin/python wheel_swap.py onshape_import/robot.xml \
    --preset gobilda_104 -o robot_fixed.xml
```

### Configure which motor is which

Your exported joints are named after your Onshape mates. Map them onto a
drivetrain the same way you would on a Driver Hub:

```bash
./.venv/bin/python configure.py onshape_import/robot.xml -o my_robot.json
./.venv/bin/python teleop.py --config my_robot.json
```

`--auto` guesses, then measures each motor's direction by driving it, so you
don't have to work out which ones need reversing.

### Gear ratios must be stated by hand

The exporter sees two rotating parts and emits two independent joints. Nothing
in the mate data says they're coupled 3:1 by a chain — a ratio is a fact about
your design, not about the geometry.

```python
from motors import YJ_312, YJ_60
drive = YJ_312.geared(2.0)   # 2:1 chain to the wheels
arm   = YJ_60.geared(3.0)    # 3:1 reduction to the arm
```

Torque up, speed down, peak power unchanged. Note this is *external* reduction
only — goBILDA quote figures at the output shaft, so a Yellow Jacket's internal
planetary is already accounted for.

---

## When it glitches, in order of likelihood

| symptom | cause |
|---|---|
| Robot is one rigid lump | Mates aren't Revolute/Slider, or aren't named `dof_` |
| Parts float away on frame 1 | Unmated parts, or more than 1 root node |
| Robot explodes instantly | Geometry starts interpenetrating — usually a convex hull filling a gap |
| Everything weighs nothing | Missing materials |
| Robot drives but cannot strafe | Mecanum wheels not swapped |
| A mechanism cannot move through its range | Convex hulls filling the space it moves through |
| Arm passes through the chassis | Needs explicit `<pair>` — MuJoCo auto-excludes body/parent contacts |
| Sim runs at 5 fps | Meshes not simplified |

That last-but-one is worth remembering: contype/conaffinity is not the only
filter. **MuJoCo automatically excludes contacts between a body and its
parent**, so no bitmask can ever make an arm collide with the chassis it is
mounted to. That needs explicit `<pair>` elements. See `collision_test.py`,
which asserts the whole matrix — a collision that silently doesn't happen raises
no error at all.

---

## Start small

Import **one mechanism** first — an arm, ten parts, two joints. You will hit
mate naming, missing materials and convex hulls on something you can actually
debug. Finding all three at once across two hundred parts is how people give up
on this.
