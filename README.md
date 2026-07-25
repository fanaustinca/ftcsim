# ftcsim

A 3D physics simulator for FIRST Tech Challenge robots, built on
[MuJoCo](https://mujoco.org/). Real mecanum physics, a motor model that
respects the torque–speed curve, gamepad or keyboard teleop, and an import path
from Onshape CAD.

**v0.9.0** — see [CHANGELOG.md](CHANGELOG.md). The running version is shown in
the bottom-right of the teleop window and by `python version.py`; it includes
the git hash, so `v0.9.0+d6c7f37-dirty` means release 0.9.0 at commit d6c7f37
with uncommitted local edits.

Built as a learning project. It is not affiliated with *FIRST* or goBILDA.

![robot](docs/robot.png)

## Status

Honest state of things, so you know what you're getting:

| Feature | State |
|---|---|
| Robot built from real aluminium channel + polycarbonate | **Working** |
| Mass computed from geometry x material density | **Working** |
| Arm torque / payload analysis | **Working** |
| Mecanum drivetrain (40 hinged rollers) | **Working**, validated |
| Motor torque–speed model, encoders, IMU | **Working** |
| Arm with gravity-feedforward hold | **Working** |
| Keyboard teleop, single window, mouse camera | **Working** |
| Gamepad teleop | **Untested** — merges with keyboard, never run against real hardware |
| Wheel presets + CAD wheel swapping | **Working** |
| Onshape import | **Wired up, never run** — needs your API keys |
| Claw picking up a game element | **Working** |
| Arm/claw collides with the robot | **Working** |
| Gyro heading-hold | **Working** |

## Install

**Full instructions: [docs/SETUP.md](docs/SETUP.md)** — including troubleshooting.

Requires Python 3.10+. On Windows, use WSL2 (WSLg supplies the GUI).

```bash
git clone https://github.com/YOUR_USERNAME/ftcsim.git
cd ftcsim
python3 -m venv .venv
./.venv/bin/pip install mujoco numpy pygame imageio imageio-ffmpeg pillow
```

Optional, only if you want Onshape import:

```bash
./.venv/bin/pip install onshape-to-robot
```

## Use

### Drive it

```bash
./.venv/bin/python teleop.py
```

One window. Everything is ours — keys, mouse, camera.

| key | action | key | action |
|---|---|---|---|
| `W` / `S` | forward / back | `SPACE` | toggle claw |
| `A` / `D` | strafe left / right | `G` | toggle field-centric |
| `Q` / `E` | turn left / right | `BKSP` | reset |
| `R` / `F` | arm up / down | `H` | gyro heading-hold |
| | | `ESC` | quit |
| left-drag | orbit camera | scroll | zoom |
| **right-drag** | **pan camera** | `HOME` | recentre on robot |
| `TAB` | cycle camera (chase / close / overhead / wide) | | |

The camera target is stored as an *offset* from the robot rather than an
absolute point, so panning survives while the robot keeps being followed —
otherwise the per-frame follow would overwrite your pan instantly. Pan distance
scales with zoom, so a drag moves the view the same amount on screen whether
you're zoomed right in or way out.

A plugged-in Xbox-style controller works *alongside* the keyboard rather than
replacing it.

> **Why no MuJoCo viewer window?** MuJoCo's built-in viewer binds every letter
> A–Z to a render flag — `W` is wireframe, `S` is shadow, `R` is reflection,
> `G` is fog. Those are handled inside its C library, so they fire whenever that
> window has focus and Python can neither intercept nor undo them: driving with
> WASD turned the robot into a wireframe grid. So we don't use it. MuJoCo
> renders offscreen, we blit the image into our own pygame window, and we own
> every event. Costs roughly 8 fps; worth it.
>
> Interactive driving uses `quality="fast"` (1024 shadow map, no floor
> reflection) for ~52 fps. Shadows and reflections together cost ~19 ms/frame.
> Video and stills use `quality="high"`.

### Validate the drivetrain

```bash
./.venv/bin/python drive_test.py
```

```
command              dX      dY    dYaw     m/s  peak rad/s
forward            1.58   -0.02     1.1    0.79        31.8
strafe left       -0.00    1.59    -0.1    0.79        31.8
strafe right       0.01   -1.59     0.0    0.79        31.7
```

Strafing is pure sideways to within 0.1° of yaw. Peak wheel speed pins at free
speed (32.7 rad/s) instead of running away.

### What does it weigh, and can the motors move it?

```bash
./.venv/bin/python mass_report.py
```

Nothing here is a typed-in number. The robot is built from goBILDA-style
aluminium U-channel and polycarbonate plate at their real densities, and MuJoCo
integrates the geometry:

```
  chassis + electronics        3.658 kg   65.8%
  wheels                       1.400 kg   25.2%
  arm assembly                 0.506 kg    9.1%
  TOTAL                        5.564 kg

  arm assembly mass         0.506 kg
  centre of gravity         265.4 mm from the shoulder pivot
  holding torque at horizontal: 1.316 N*m

  motor                           stall    margin  verdict
  goBILDA 60 RPM (99.5:1)         6.38N*m      4.8x  comfortable
  goBILDA 312 RPM (19.2:1)        2.04N*m      1.5x  workable, run a hold PID
```

The gravity torque comes from MuJoCo's own `qfrc_bias`, and it agrees with the
hand calculation m*g*r to three decimals. Change `ARM_LEN` in `robot.py` and
every number above moves with it — which is the entire point.

A U-channel is modelled as three thin boxes, not a solid block: a solid box of
the same envelope weighs roughly four times what the real extrusion does, and
that error would propagate straight into every torque result.

**Caveats.** This models frame, deck, motors, wheels, hub and battery only — a
real robot adds brackets, fasteners, wiring and mechanism hardware, so expect
the physical thing to land 2-3x heavier. Pattern holes aren't modelled, which
pushes ~10-15% the other way. Treat computed masses as good to about 15%. Weigh
a real part and adjust the density if you want better.

### Compare wheels

```bash
./.venv/bin/python compare_wheels.py
```

Measures acceleration, top speed, strafe efficiency and pushing power for each
preset. The interesting result: in free driving the robot is **motor-limited,
not traction-limited**, so roller compound barely matters. It only shows up in
a pushing match, where grip is the binding constraint.

### Record a video

```bash
./.venv/bin/python teleop_showcase.py   # scripted driving -> teleop_demo.mp4
./.venv/bin/python auto_demo.py         # scripted autonomous -> demo.mp4
```

## Wheel presets

```bash
./.venv/bin/python wheels.py     # list them
```

| key | wheel | rollers | durometer |
|---|---|---|---|
| `gobilda_96` | goBILDA 96 mm Mecanum | 10 | 70A |
| `gobilda_104` | goBILDA 104 mm GripForce | 11 | 40A |
| `soft_test` | hypothetical soft | 10 | ~30A |
| `hard_test` | hypothetical hard | 10 | ~90A |

```python
from robot import build_mjcf
xml = build_mjcf("gobilda_104")
```

Diameter, roller count and durometer for the goBILDA wheels come from published
specs. **The friction coefficients are estimates** — durometer is a hardness
rating, not a friction coefficient, and there's no clean conversion. They're
ordered correctly relative to each other, which is what matters for comparing
designs, but don't treat an absolute number as truth. Measure your real robot's
acceleration and tune `roller_friction` until the sim matches.

Fitting a bigger wheel raises the chassis, because the axle stays put — same as
on real hardware. That raises your centre of gravity.

## Importing your own robot from Onshape

**Read [docs/CAD_PREP.md](docs/CAD_PREP.md) BEFORE you start mating your
assembly.** Almost all the work is on the Onshape side, and fixing mates
afterwards means redoing them. It covers how the exporter decides what is
attached to what, the preparation checklist, the convex-collision trap, and a
symptom-to-cause table for when it glitches.

Short version:

1. Get free API keys at <https://dev-portal.onshape.com/keys>, export
   `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY`.
2. In Onshape, rename the mates that should move: a Revolute mate named
   `dof_shoulder` becomes an actuated joint. Prefixes are `dof_`, `fix_`,
   `frame_`, `link_`, `closing_`.
3. **Assign a material to every part** — mass and inertia come from there.
4. Paste your assembly URL into `onshape_import/config.json`, then
   `./.venv/bin/onshape-to-robot onshape_import`.

### Then swap the wheels

CAD wheels are the one part of an import that's actively wrong: a mecanum wheel
in CAD is a rigid lump, its rollers modelled as shapes rather than bodies that
spin. Simulate that and the robot **cannot strafe at all**.

```bash
./.venv/bin/python wheel_swap.py imported.xml --list
./.venv/bin/python wheel_swap.py imported.xml --preset gobilda_104 -o robot_fixed.xml
```

Measured on a test export, before and after:

| | forward | strafe |
|---|---|---|
| raw CAD export | 2.92 m | **0.014 m** |
| after swap | 2.74 m | **−2.36 m** |

Detected wheels are replaced with generated ones whose rollers are real hinged
bodies, and the joints come out named `drive_fl` … `drive_br` so `motors.py`
works unchanged.

## Collisions

Two filters decide whether a pair of geoms can touch, and both bit me:

1. **contype / conaffinity bitmasks.** Geoms collide when
   `(contype1 & conaffinity2) || (contype2 & conaffinity1)`. Putting every
   robot part in one bucket to stop the wheel rollers grinding on the frame
   also silently disabled arm-vs-chassis. `structure.py` now uses separate
   groups (WORLD / CHASSIS / ARM / WHEEL / GAME) so exactly one pair is
   disabled.
2. **MuJoCo automatically excludes a body from its own parent.** The arm is a
   child of the chassis, so *no bitmask can ever make them collide*. That needs
   explicit `<pair>` elements, which bypass every filter. This is why the arm
   swung through the deck no matter what the masks said.

`collision_test.py` asserts the whole matrix plus the behaviour, because a
collision that silently does not happen produces no error at all.

## How it fits together

```
robot.py       generates the MJCF model (field, chassis, arm) from Python
wheels.py      mecanum presets + the roller/wheel MJCF generator
structure.py   material densities + U-channel/plate/motor part builders
mass_report.py mass budget, arm CG, holding torque, payload capacity
collision_test.py  asserts every collision pair and that the arm stops on the deck
motors.py      torque-speed motor model + an FTC-shaped robot API
teleop.py      gamepad / keyboard driving
wheel_swap.py  detect and replace CAD wheels in an imported model
```

The motor model is the part that matters most:

```python
T = T_stall * (power - omega / omega_free)
```

A real DC motor can't produce stall torque at full speed — back-EMF eats it.
MuJoCo's `motor` actuator has no such limit: ask for 3 N·m and it delivers 3 N·m
at 8000 RPM. Without this the wheels spun to 798 rad/s with zero traction and
every mechanism "worked".

## Known issues

- **Strafe yaw is sensitive to centre-of-gravity height.** Raising the arm
  pivot 130 mm (needed so the arm can reach the floor) took strafe drift from
  under a degree to as much as 20° over two seconds, and it varies with tiny
  perturbations. That is real — tall mecanum robots do this. `drive_held()`
  applies gyro heading-hold and brings it back to ~0.2°; leave it on.
- **Gamepad path is untested.** Written, never run against a real controller.
- **Mecanum slip is directionally right, quantitatively wrong.** Fine for
  teleop feel; don't trust it for precise autonomous.
- **No belt/chain compliance or backlash.** Sim joints are rigid; your real arm
  has slop.
- **Battery sag is a crude global scale factor**, not a real model.

## Licence

MIT
