# Changelog

Versions follow [semantic versioning](https://semver.org/): `MAJOR.MINOR.PATCH`.

Still `0.x` — the API isn't promised to be stable. Two things block `1.0.0`:
the gamepad path has never run against a real controller, and the Onshape
import has never run against a real document.

The running version is shown in the bottom-right of the teleop window, and
`python version.py` prints it. It includes the git hash, so
`v0.9.0+d6c7f37-dirty` means "release 0.9.0, commit d6c7f37, with uncommitted
local edits."

---

## 0.13.0

You no longer need to know which Onshape mate is which wheel.

- `configure.py --auto` identifies wheels from **geometry, not names**. Wheels
  are the lowest joints on the robot; once you have them, front/back and
  left/right fall out of position. Verified against a model whose joints are
  called `Revolute_1`..`Revolute_4` in an order unrelated to the layout — all
  four corners identified correctly
- Prints an ASCII map of where each wheel is, so you can check it by eye
- `--forward {+x,-x,+y,-y}` for CAD modelled facing a different direction.
  Verified front/back swap when it flips
- Name matching is now only a fallback for when the model can't be loaded

## 0.12.0

- `import_check.py` — health-checks an exported robot before you try to drive
  it. Every failure mode in CAD_PREP.md is silent: a robot with no materials
  loads fine and weighs nothing, an unmated part loads fine and floats away, a
  model with no `dof_` mates loads fine and is one rigid lump. This names which
  one you have
- Checks: rigid lump, disconnected bodies, missing materials, degenerate
  inertia, missing actuators, rigid mecanum rollers, geometry that starts
  interpenetrating, and whether it settles or explodes
- Rigid-roller detection reads GEOM names as well as body names, because a CAD
  export models rollers as geometry inside a rigid wheel body — there is no
  roller *body* to find. Without that it passed a robot that could not strafe
- Free-body detection ranks by subtree mass instead of counting, so a legitimate
  game element isn't reported as the robot being in pieces

## 0.11.0

Robot configuration, the way a Driver Hub does it.

- `configure.py` — interactive wizard: pick a drivetrain, then for each wheel
  say which hub port it's on, what it's called, and whether it's reversed.
  `--auto` guesses it all instead of prompting
- `robotconfig.py` — `RobotConfig` saved as JSON, with validation that catches
  duplicate ports, duplicate joints, and unfilled drivetrain positions
- Drivetrains: 4-wheel mecanum, 4- and 6-wheel tank, 2-wheel tank. Tank
  layouts correctly ignore sideways input, same as the real thing
- `Robot(model, data, config=...)` maps positions onto whatever the joints are
  actually called — which is what makes a CAD import usable, since after export
  the joints are named after your Onshape mates
- `teleop.py --config robot_config.json`
- **Motor reversal is measured, not guessed.** `--auto` drives each motor on
  its own and watches which way the robot goes. Guessing from names ("right
  side is usually reversed") is right on real hardware and wrong on a model
  whose joints share an axis — the name-based guess would have reversed the
  right side and broken the built-in robot
- Fixed: auto-assign grabbed unrelated joints when a drivetrain needed more
  wheels than the model had, silently making `shoulder` and `wrist` into drive
  motors on a 6WD config. It now leaves them unassigned so validation reports it
- Fixed: ports were indexed by wheel position, so an unmatched wheel left a gap
  that the mechanism loop then reused, double-booking a port

## 0.10.0

Claw tilt control — you can now pick the block up by hand, not just in the
scripted routine.

- `Z` / `X` tilt the claw down and up. Teleop previously locked the wrist level
  (`level_wrist(0.0)`), which made a floor pickup **geometrically impossible**:
  the wrist block grounds out on the deck with the claw tip at 0.083 m, and the
  game element's top is at 0.076 m. Tilted down, the arm reaches its joint limit
  with the tip at 0.006 m instead
- `1` snaps to the grab pose (arm down, claw pointed at the floor, open) and `2`
  returns to travel pose. Same measured values `auto_demo.py` uses
- Claw tilt shown in the HUD
- Verified end to end by simulating the key presses: drive up, `1`, `SPACE`,
  `2` — cube lifts to 0.558 m and stays in the claw

## 0.9.1

- Fixed the version string overlapping the controls hint at the bottom of the
  teleop window. The hint now trims itself from the end to fit the space left
  by the version box, rather than both drawing at fixed positions

## 0.9.0

Version tracking.

- `version.py` as the single source of truth, shown in the teleop HUD and the
  window title
- Version string includes the short git hash and a `-dirty` marker, because the
  constant only changes when someone remembers to bump it while the hash
  changes every commit
- This changelog

## 0.8.0

Documentation.

- `docs/SETUP.md` — install from scratch, a verification step with expected
  numbers, and troubleshooting for the common WSL/EGL/venv failures
- `docs/CAD_PREP.md` — preparing an Onshape assembly: how the exporter builds
  the kinematic tree from mate names, the checklist, the convex-collision trap,
  and a symptom-to-cause table
- `link_hull_test.py` — measured "one collision shape per link instead of per
  part": saves 25% of collision geoms and destroys 114° of arm travel, because
  a link's convex hull swallows its own working volume

## 0.7.0

Gear ratios.

- `Motor.geared(ratio)` for reductions between motor and mechanism — chain,
  belt, gear pair. Verified peak mechanical power is unchanged at 18.6 W across
  1:1, 2:1 and 4:1: gearing trades torque for speed, it doesn't create power
- Separate from the motor's internal gearbox, which goBILDA already fold into
  their published output-shaft figures
- `convex_demo.py` — measured that hull collision inflates robot mass 63.7%
- Removed an earlier synthetic "arm in a cage" demo rather than publish it:
  disabling its contact pairs changed nothing, proving it measured something
  other than the cage

## 0.6.0

Collisions, arm tower, heading hold.

- **Fixed: the arm passed through the chassis.** Two independent causes. The
  collision bitmasks lumped all robot parts together, and separately MuJoCo
  automatically excludes contacts between a body and its parent — which no
  bitmask can override. Fixed with per-group masks plus explicit `<pair>`
  elements
- **Fixed: dropped game elements fell through the robot.** Control Hub, battery
  and motors were `contype=0`, entirely non-colliding
- Added a 160 mm arm tower. With collisions enabled the arm bottomed out on the
  deck at −0.05 rad; it now reaches −0.42 rad and picks up the block again
- Added `drive_held()` — gyro heading hold. The taller tower raised the centre
  of gravity and strafe drift went from under 1° to as much as 20° over two
  seconds; this brings it back to ~0.2°. On by default, `H` toggles
- `collision_test.py` asserts the whole collision matrix, since a collision
  that silently doesn't happen raises no error

## 0.5.0

- Right-drag camera panning, `HOME` to recentre. Stored as an offset from the
  robot so it survives the per-frame follow

## 0.4.0

- **Dropped MuJoCo's viewer entirely.** It binds all 26 letters to render flags
  (`W` is wireframe), handled inside its C library where Python can't intercept
  it — so driving with WASD strobed the render settings. Now MuJoCo renders
  offscreen and we blit into our own pygame window, owning every event
- Our own camera: mouse orbit, scroll zoom, `TAB` cycles four presets
- `build_mjcf(quality=...)`: `fast` runs 52 fps against `high`'s 34

## 0.3.0

- **Fixed: holding a key gave discrete nudges.** The loop did one physics step
  plus one sleep per iteration at 500 Hz, which is unreliable; physics and
  rendering are now decoupled by a wall-clock accumulator
- **Fixed: WASD could be swallowed** whenever any joystick was present, even an
  idle one
- **Fixed: the deck floated 66 mm above the frame** with nothing between them
- Rebuilt the claw as aluminium plates with grip pads instead of two capsules

## 0.2.0

- **Robot built from real structure** — aluminium U-channel and polycarbonate at
  real densities, so mass is integrated from geometry rather than typed in.
  Total went 12.88 → 5.66 kg because the old figure was invented
- A U-channel is three thin boxes, not a solid block: a solid box of the same
  envelope weighs ~4× the real extrusion
- `mass_report.py` — mass budget, arm centre of gravity, holding torque versus
  motor stall, payload capacity. Gravity torque from MuJoCo's `qfrc_bias` agrees
  with `m·g·r` to three decimals

## 0.1.0

Initial.

- Mecanum drivetrain with 40 individually-hinged rollers; strafing pure to
  within 0.1° of yaw
- DC motor torque–speed model. Without it, wheels ran to 798 rad/s with no
  traction and every mechanism "worked"
- Arm PD hold with `cos(θ)` gravity feedforward, wrist levelling
- Wheel presets with a measured comparison harness
- `wheel_swap.py` — replaces rigid CAD wheels from an import with generated
  hinged-roller wheels. A raw CAD export cannot strafe at all: 0.014 m versus
  2.36 m after the swap
- Onshape import configuration
