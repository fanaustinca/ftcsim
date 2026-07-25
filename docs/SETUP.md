# Setup

From nothing to a driving robot. Roughly 10 minutes, most of it downloads.

## What you need

- **Python 3.10 or newer**
- **Windows: use WSL2.** WSLg supplies the graphics; everything below assumes
  you're inside the WSL shell, not PowerShell.
- A GPU helps but isn't required. Software rendering works, just slower.

Check Python:

```bash
python3 --version
```

If that errors or shows 3.9 or older:

```bash
sudo apt update && sudo apt install python3 python3-venv python3-pip
```

## Install

```bash
git clone https://github.com/fanaustinca/ftcsim.git
cd ftcsim
python3 -m venv .venv
./.venv/bin/pip install --upgrade pip
./.venv/bin/pip install mujoco numpy pygame imageio imageio-ffmpeg pillow
```

Only if you want to import your own CAD:

```bash
./.venv/bin/pip install onshape-to-robot
```

Everything runs through `./.venv/bin/python`. You never need to "activate" the
venv, and using plain `python3` will fail with import errors — that's the most
common setup mistake.

## Which version am I on?

```bash
./.venv/bin/python version.py
```

```
v0.9.0+d6c7f37
```

Release, then the git commit. A `-dirty` suffix means you have uncommitted
local edits. The same string sits in the bottom-right corner of the teleop
window, so you can always tell what you're looking at. Quote it if you report a
problem.

## Check it works

```bash
./.venv/bin/python drive_test.py
```

Expected — strafing should be pure sideways, with under a degree of yaw:

```
command              dX      dY    dYaw     m/s  peak rad/s
forward            1.63    0.00    -0.2    0.81        32.2
strafe left       -0.03    1.58    -0.5    0.79        31.8
strafe right      -0.00   -1.58     0.1    0.79        31.8
```

If the numbers are close to those, the physics is working. Then:

```bash
./.venv/bin/python teleop.py
```

A window opens. Click it, then `W`/`A`/`S`/`D`.

## Controls

| key | action | key | action |
|---|---|---|---|
| `W` / `S` | forward / back | `SPACE` | toggle claw |
| `A` / `D` | strafe left / right | `G` | field-centric |
| `Q` / `E` | turn left / right | `H` | gyro heading-hold (default on) |
| `R` / `F` | arm up / down | `BKSP` | reset |
| left-drag | orbit camera | right-drag | pan camera |
| scroll | zoom | `TAB` | cycle camera |
| `HOME` | recentre | `ESC` | quit |

Plug in an Xbox-style controller before launching and it works alongside the
keyboard.

## Everything else

```bash
./.venv/bin/python mass_report.py      # what it weighs, can the motors move it
./.venv/bin/python compare_wheels.py   # wheel presets measured against each other
./.venv/bin/python collision_test.py   # asserts every collision pair
./.venv/bin/python auto_demo.py        # scripted pickup -> demo.mp4
./.venv/bin/python teleop_showcase.py  # scripted driving -> teleop_demo.mp4
./.venv/bin/python wheels.py           # list wheel presets
```

## Troubleshooting

**`ModuleNotFoundError: No module named 'mujoco'`**
You ran `python3` instead of `./.venv/bin/python`.

**No window appears (WSL)**
Check WSLg is alive: `echo $DISPLAY` should print something like `:0`. If it's
empty, update WSL from PowerShell with `wsl --update` and restart it.

**Window opens black, or an EGL error on startup**
Force software rendering:
```bash
MUJOCO_GL=osmesa ./.venv/bin/python teleop.py
```
Slower but reliable.

**Runs but feels sluggish**
`teleop.py` already uses `quality="fast"`. Drop the window size — edit
`WIN_W, WIN_H` near the top of `teleop.py` to `900, 560`. Render cost scales
with pixel count, and rendering is the bottleneck, not physics.

**Gamepad not detected**
USB passthrough to WSL is genuinely awkward. The keyboard path is fully
supported; use it unless you specifically need analog sticks.

**Tracebacks mentioning EGL when a script finishes**
Harmless. That's OpenGL teardown noise after the work is done.

## Importing your own robot

See [CAD_PREP.md](CAD_PREP.md). Read it *before* you start mating your
assembly — most of the work is preparation, and doing it after the fact means
redoing mates.
