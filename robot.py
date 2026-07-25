"""
Builds an FTC-style robot + field as a MuJoCo model.

We generate the MJCF (MuJoCo's XML format) from Python instead of writing it by
hand, because the mecanum wheels need ~10 separately-hinged rollers each and
that's 40 near-identical blocks of XML. Generating it also means changing a
dimension is a one-line edit instead of a find-and-replace.

Everything is SI units: meters, kilograms, seconds, radians.
"""

import numpy as np

import wheels

# ---------------------------------------------------------------------------
# Dimensions. Tweak these; everything downstream follows.
# ---------------------------------------------------------------------------

FIELD_SIZE = 3.6576        # 12 ft, the real FTC field
WALL_HEIGHT = 0.30
TILE = 0.6096              # 24" foam tiles, used for the floor texture scale

CHASSIS_L = 0.36           # just under the 18" starting cube
CHASSIS_W = 0.36
CHASSIS_H = 0.09
CHASSIS_MASS = 10.0
RIDE_HEIGHT = 0.055        # chassis underside above the floor

WHEEL_R = 0.048            # nominal; the active preset overrides this
WHEEL_X = 0.13             # forward/back offset from robot center
WHEEL_Y = 0.20             # left/right offset


ARM_LEN = 0.36
FINGER_LEN = 0.07

# Wheel order is fixed everywhere in this project: front-left, front-right,
# back-left, back-right. Mecanum roller handedness alternates diagonally --
# FL and BR share one direction, FR and BL the other. Getting this wrong is the
# single most common mecanum bug, in sim and on real hardware.
WHEELS = [
    # name,  x,         y,         roller handedness
    ("fl",  +WHEEL_X, +WHEEL_Y, +1),
    ("fr",  +WHEEL_X, -WHEEL_Y, -1),
    ("bl",  -WHEEL_X, +WHEEL_Y, -1),
    ("br",  -WHEEL_X, -WHEEL_Y, +1),
]


def build_mjcf(wheel_preset=wheels.DEFAULT, push_block_kg=0.0):
    """Build the model.

    wheel_preset   which mecanum wheel is fitted (see wheels.PRESETS)
    push_block_kg  if >0, drops a heavy block in front of the robot. Driving
                   into it is a pushing match, which is traction-limited rather
                   than motor-limited -- the only way roller grip shows up in
                   the numbers.
    """
    preset = wheels.get(wheel_preset) if isinstance(wheel_preset, str) else wheel_preset

    # The axle sits at a fixed height on the chassis, so fitting a bigger wheel
    # physically LIFTS the robot -- same as it would on real hardware. That
    # raises the centre of gravity, which is a real consequence of the swap and
    # something the sim should show you rather than hide.
    push_block = "" if push_block_kg <= 0 else f"""
    <body name="pushblock" pos="0.75 0 0.12">
      <freejoint/>
      <geom name="pushblockg" type="box" size="0.18 0.18 0.12"
            mass="{push_block_kg}" contype="1" conaffinity="1"
            friction="0.9 0.01 0.001" rgba="0.75 0.25 0.30 1"/>
    </body>"""

    wheel_z = -CHASSIS_H / 2
    chassis_z = preset.radius + CHASSIS_H / 2 + 0.002
    wheels_xml = "".join(
        wheels.wheel_xml(preset, n, x, y, wheel_z, h) for n, x, y, h in WHEELS)
    half = FIELD_SIZE / 2

    walls = ""
    for i, (wx, wy, sx, sy) in enumerate([
        (+half, 0, 0.02, half), (-half, 0, 0.02, half),
        (0, +half, half, 0.02), (0, -half, half, 0.02),
    ]):
        walls += f"""
    <geom name="wall{i}" type="box" pos="{wx} {wy} {WALL_HEIGHT/2}"
          size="{sx} {sy} {WALL_HEIGHT/2}" contype="1" conaffinity="1"
          rgba="0.15 0.17 0.22 1"/>"""

    return f"""
<mujoco model="ftc_sim">
  <compiler angle="radian" autolimits="true"/>

  <!-- elliptic friction cone is noticeably better for wheel contact than the
       default pyramidal one; it costs a little speed and is worth it here. -->
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.4 0.4 0.4"/>
    <global azimuth="140" elevation="-25" offwidth="1920" offheight="1080"/>
    <quality shadowsize="4096"/>
  </visual>

  <asset>
    <texture name="sky" type="skybox" builtin="gradient"
             rgb1="0.22 0.26 0.34" rgb2="0.05 0.06 0.09" width="512" height="512"/>
    <texture name="tiles" type="2d" builtin="checker"
             rgb1="0.22 0.24 0.28" rgb2="0.17 0.19 0.23" width="512" height="512"/>
    <material name="floor" texture="tiles" texrepeat="{FIELD_SIZE/TILE:.4f} {FIELD_SIZE/TILE:.4f}"
              reflectance="0.05"/>
  </asset>

  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" diffuse="0.7 0.7 0.7" castshadow="true"/>
    <geom name="floor" type="plane" size="{half} {half} 0.1" material="floor"
          contype="1" conaffinity="1" friction="1.0 0.005 0.0001"/>
    {walls}

    {push_block}

    <!-- game element: something to actually pick up -->
    <body name="cube" pos="0.9 0.0 0.04">
      <freejoint/>
      <geom name="cubeg" type="box" size="0.038 0.038 0.038" mass="0.10"
            contype="1" conaffinity="1" friction="1.0 0.01 0.001" rgba="0.95 0.62 0.15 1"/>
    </body>

    <body name="chassis" pos="0 0 {chassis_z:.5f}">
      <freejoint/>
      <geom name="chassisg" type="box" size="{CHASSIS_L/2} {CHASSIS_W/2} {CHASSIS_H/2}"
            mass="{CHASSIS_MASS}" contype="2" conaffinity="1" rgba="0.20 0.42 0.75 1"/>
      <site name="imu" pos="0 0 0" size="0.01"/>
      {wheels_xml}

      <!-- Arm: shoulder pivot at the back, wrist at the far end, 2-finger claw.
           Deliberately simple -- this is the "one mechanism that works" shape. -->
      <body name="arm" pos="{-CHASSIS_L/2 + 0.05} 0 {CHASSIS_H/2}">
        <!-- axis is -Y so that positive angle raises the arm, which is what
             anyone reading the code will assume. Lower limit reaches the floor
             out front; upper limit tucks it back over the chassis. -->
        <joint name="shoulder" type="hinge" axis="0 -1 0" range="-0.45 2.00" damping="0.6"/>
        <geom name="armg" type="capsule" size="0.022" fromto="0 0 0 {ARM_LEN} 0 0"
              mass="1.1" contype="2" conaffinity="1" rgba="0.75 0.78 0.82 1"/>

        <body name="wrist" pos="{ARM_LEN} 0 0">
          <joint name="wrist" type="hinge" axis="0 1 0" range="-1.7 1.7" damping="0.12"/>
          <geom name="wristg" type="box" size="0.028 0.045 0.014" mass="0.18"
                contype="2" conaffinity="1" rgba="0.55 0.58 0.62 1"/>

          <body name="finger_l" pos="0.026 0.055 0">
            <joint name="grip_l" type="hinge" axis="0 0 1" range="-0.40 0.40" damping="0.05"/>
            <geom name="finger_lg" type="capsule" size="0.008"
                  fromto="0 0 0 {FINGER_LEN} 0 0" mass="0.05"
                  contype="2" conaffinity="1" friction="3.5 0.05 0.005" rgba="0.90 0.35 0.30 1"/>
          </body>
          <body name="finger_r" pos="0.026 -0.055 0">
            <joint name="grip_r" type="hinge" axis="0 0 1" range="-0.40 0.40" damping="0.05"/>
            <geom name="finger_rg" type="capsule" size="0.008"
                  fromto="0 0 0 {FINGER_LEN} 0 0" mass="0.05"
                  contype="2" conaffinity="1" friction="3.5 0.05 0.005" rgba="0.90 0.35 0.30 1"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- Plain torque sources for now. Phase 2 wraps these in a real motor model
         so that stall torque isn't available at full speed. -->
    <motor name="m_fl" joint="drive_fl" gear="1" ctrlrange="-6 6"/>
    <motor name="m_fr" joint="drive_fr" gear="1" ctrlrange="-6 6"/>
    <motor name="m_bl" joint="drive_bl" gear="1" ctrlrange="-6 6"/>
    <motor name="m_br" joint="drive_br" gear="1" ctrlrange="-6 6"/>
    <motor name="m_shoulder" joint="shoulder" gear="1" ctrlrange="-25 25"/>

    <!-- Servos are position-controlled, which is what real FTC servos are. -->
    <position name="s_wrist" joint="wrist" kp="12" dampratio="1" ctrlrange="-1.7 1.7"/>
    <position name="s_grip_l" joint="grip_l" kp="45" dampratio="1" ctrlrange="-0.40 0.40"/>
    <position name="s_grip_r" joint="grip_r" kp="45" dampratio="1" ctrlrange="-0.40 0.40"/>
  </actuator>

  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
    <gyro name="imu_gyro" site="imu"/>
  </sensor>
</mujoco>
"""


if __name__ == "__main__":
    print(build_mjcf())
