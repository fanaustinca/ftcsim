# Importing a real Onshape robot

`onshape-to-robot` (already installed in `../.venv`) pulls an assembly straight
from Onshape's API and writes MuJoCo MJCF, including meshes, masses and inertia
tensors taken from the materials you assigned in CAD.

## 1. Get API keys

Go to <https://dev-portal.onshape.com/keys>, sign in with your normal Onshape
account, create a key pair, then:

```bash
export ONSHAPE_ACCESS_KEY=your_access_key
export ONSHAPE_SECRET_KEY=your_secret_key
```

Put those in `~/.bashrc` so you don't redo it every session. A free personal
Onshape account can create dev keys — you do not need a paid plan.

## 2. Prepare the assembly in Onshape

This is the part that takes real time, and it is done in Onshape, not here.
The exporter decides what is a joint purely from **mate names**:

| prefix       | meaning                                                    |
|--------------|------------------------------------------------------------|
| `dof_name`   | an actuated joint — revolute or slider becomes a MuJoCo joint |
| `fix_name`   | rigidly welded, no joint created                            |
| `frame_name` | a named reference frame (handy for sensor/camera mounts)    |
| `link_name`  | names the resulting link                                    |
| `closing_x`  | closes a kinematic loop (four-bar linkages, etc.)           |

So a shoulder pivot becomes a **Revolute** mate renamed `dof_shoulder`.

Two things that will bite you:

- **Assign a material to every part.** Mass and inertia come from Onshape's
  material properties. Parts left as "no material" export with junk mass, and
  the robot will behave wrongly in ways that are hard to trace back.
- **Use real Revolute/Slider mates for anything that moves.** If a joint is
  held by Fastened or coincident/tangent mates, the exporter cannot tell it is
  a joint and you get one welded block.

## 3. Export

```bash
cd onshape_import
# paste your assembly URL into config.json first
../.venv/bin/onshape-to-robot .
```

Output lands in this directory as `robot.xml` plus an `assets/` folder of
meshes.

## 4. Load it

```python
import mujoco
model = mujoco.MjModel.from_xml_path("onshape_import/robot.xml")
```

`motors.py` works unchanged as long as your joints are named the same way the
hand-built model names them (`drive_fl`, `shoulder`, ...). If you name the
Onshape mates `dof_drive_fl` and so on, they line up automatically.

## What still needs doing by hand after import

The export gives you the robot. It does not give you:

- **The field** — walls, tiles, game elements. Keep authoring those in
  `robot.py`; there is no reason to CAD a field.
- **Actuators** — the exporter creates joints, not motors. You add
  `<motor>`/`<position>` entries and point `motors.py` at them.
- **Mecanum rollers** — CAD rollers export as static geometry, not free-spinning
  bodies. The generated rollers in `robot.py` are better; keep using them and
  export the rest of the robot around them.
