"""Drive the robot yourself. One window, no surprises.

    python teleop.py

    W / S      forward / back          SPACE   toggle claw
    A / D      strafe left / right     G       field-centric on/off
    Q / E      turn left / right       BKSP    reset robot
    H          gyro heading-hold
    R / F      arm up / down           ESC     quit

    left drag    orbit camera          scroll  zoom
    right drag   pan camera             HOME    recentre on robot
    TAB          cycle camera preset

Plug in an Xbox-style controller and it works alongside the keyboard: left
stick drives, right stick X turns, triggers work the arm, A/B the claw.

WHY THERE IS NO MUJOCO VIEWER WINDOW
    MuJoCo's built-in viewer binds every letter A-Z to a render flag -- W is
    wireframe, S is shadow, R is reflection, G is fog. Those are handled inside
    its C library, so they fire whenever that window has focus and Python cannot
    intercept or undo them. Driving with WASD anywhere near it turns the robot
    into a wireframe grid.

    So we don't use it. MuJoCo renders offscreen, we blit the image into our own
    pygame window, and we own every key and mouse event. The camera controls
    below are ours too. Costs a few frames per second; worth it.
"""
import os

os.environ.setdefault("MUJOCO_GL", "egl")

import time

import mujoco
import numpy as np
import pygame

from robot import build_mjcf
from motors import Robot

WIN_W, WIN_H = 1180, 740
DEADZONE = 0.08
FRAME = 1.0 / 60.0

# name, distance, elevation, azimuth-follows-robot
CAMERAS = [
    ("chase",   1.55, -18, True),
    ("close",   0.95, -14, True),
    ("overhead", 2.60, -78, False),
    ("wide",    3.20, -28, False),
]


def deadzone(v):
    return 0.0 if abs(v) < DEADZONE else (abs(v) - DEADZONE) / (1 - DEADZONE) * np.sign(v)


class Sim:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("ftcsim - teleop")
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        self.font = pygame.font.SysFont("monospace", 15)
        self.big = pygame.font.SysFont("monospace", 19, bold=True)

        pygame.joystick.init()
        self.js = None
        if pygame.joystick.get_count():
            self.js = pygame.joystick.Joystick(0)
            self.js.init()

        self.model = mujoco.MjModel.from_xml_string(build_mjcf(quality="fast"))
        self.data = mujoco.MjData(self.model)
        self.bot = Robot(self.model, self.data)
        self.bot.stow()

        self.renderer = mujoco.Renderer(self.model, height=WIN_H, width=WIN_W)
        self.cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.cam)
        self.cam_idx = 0
        self.az_offset = 0.0
        self.pan = np.zeros(3)      # camera target offset from the robot
        self.el = CAMERAS[0][2]
        self.dist = CAMERAS[0][1]

        self.grip = 0.0
        self.grip_closed = False
        self.field_centric = False
        self.heading_hold = True
        self.arm_target = 1.2
        self.manual_arm = False
        self.running = True
        self.fps = 0.0

    # -- input ----------------------------------------------------------

    def handle_events(self):
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                self.running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    self.running = False
                elif e.key == pygame.K_SPACE:
                    self.grip_closed = not self.grip_closed
                elif e.key == pygame.K_g:
                    self.field_centric = not self.field_centric
                elif e.key == pygame.K_h:
                    self.heading_hold = not self.heading_hold
                    self.bot.heading_target = None
                elif e.key == pygame.K_BACKSPACE:
                    mujoco.mj_resetData(self.model, self.data)
                    self.bot.stow()
                    self.arm_target, self.grip = 1.2, 0.0
                elif e.key == pygame.K_TAB:
                    self.cam_idx = (self.cam_idx + 1) % len(CAMERAS)
                    _, self.dist, self.el, _ = CAMERAS[self.cam_idx]
                    self.az_offset = 0.0
                    self.pan[:] = 0.0
                elif e.key == pygame.K_HOME:
                    self.pan[:] = 0.0
                    self.az_offset = 0.0
            elif e.type == pygame.MOUSEWHEEL:
                self.dist = float(np.clip(self.dist * (0.9 ** e.y), 0.4, 6.0))
            elif e.type == pygame.MOUSEMOTION and e.buttons[0]:
                self.az_offset += e.rel[0] * 0.4
                self.el = float(np.clip(self.el - e.rel[1] * 0.3, -89, 5))
            elif e.type == pygame.MOUSEMOTION and (e.buttons[2] or e.buttons[1]):
                self.pan_camera(e.rel[0], e.rel[1])
            elif e.type == pygame.JOYBUTTONDOWN and self.js:
                if e.button == 0:
                    self.grip_closed = True
                elif e.button == 1:
                    self.grip_closed = False
                elif e.button == 2:
                    self.field_centric = not self.field_centric

    def sticks(self):
        k = pygame.key.get_pressed()
        axis = lambda a, b: float(k[a]) - float(k[b])
        out = dict(
            fwd=axis(pygame.K_w, pygame.K_s),
            strafe=axis(pygame.K_d, pygame.K_a),
            turn=axis(pygame.K_e, pygame.K_q),
            arm=axis(pygame.K_r, pygame.K_f),
        )
        # Gamepad ADDS to the keyboard rather than replacing it, so an idle or
        # phantom controller can't silently swallow WASD.
        if self.js:
            ax = lambda i: deadzone(self.js.get_axis(i)) if self.js.get_numaxes() > i else 0.0
            lt = (self.js.get_axis(4) + 1) / 2 if self.js.get_numaxes() > 4 else 0.0
            rt = (self.js.get_axis(5) + 1) / 2 if self.js.get_numaxes() > 5 else 0.0
            for key, v in dict(fwd=-ax(1), strafe=ax(0), turn=ax(3), arm=rt - lt).items():
                if abs(v) > abs(out[key]):
                    out[key] = v
        return out

    def pan_camera(self, dx, dy):
        """Slide the camera sideways/up-down in its own screen plane.

        The target is stored as an OFFSET from the robot, not an absolute
        point, because draw() re-aims the camera at the chassis every frame --
        an absolute target would be overwritten instantly. Storing an offset
        means panning survives while the robot keeps being followed.

        Scaled by distance so a drag moves the view the same amount on screen
        whether you're zoomed right in or way out.
        """
        az = np.radians(self.cam.azimuth)
        el = np.radians(self.cam.elevation)

        # Screen-plane basis for the current camera orientation.
        right = np.array([-np.sin(az), np.cos(az), 0.0])
        up = np.array([-np.sin(el) * np.cos(az),
                       -np.sin(el) * np.sin(az),
                       np.cos(el)])

        scale = self.dist * 0.0016
        # Drag the world with the cursor: moving right pushes the target left.
        self.pan -= right * dx * scale
        self.pan += up * dy * scale

    # -- physics --------------------------------------------------------

    def step_physics(self, s, n):
        for _ in range(n):
            fwd, strafe = s["fwd"], s["strafe"]
            if self.field_centric:
                h = self.bot.heading()
                fwd, strafe = (fwd * np.cos(-h) - strafe * np.sin(-h),
                               fwd * np.sin(-h) + strafe * np.cos(-h))
            if self.heading_hold:
                self.bot.drive_held(fwd, strafe, s["turn"])
            else:
                self.bot.drive(fwd, strafe, s["turn"])

            if abs(s["arm"]) > 0.05:
                self.manual_arm = True
                self.bot.set_arm(s["arm"])
            else:
                if self.manual_arm:
                    self.arm_target = float(self.data.joint("shoulder").qpos[0])
                    self.manual_arm = False
                self.bot.hold_arm(self.arm_target)
            self.bot.level_wrist(0.0)

            self.grip = (min(1.0, self.grip + 0.02) if self.grip_closed
                         else max(0.0, self.grip - 0.02))
            self.bot.set_grip(self.grip)

            self.bot.apply()
            mujoco.mj_step(self.model, self.data)

    # -- drawing --------------------------------------------------------

    def draw(self):
        name, _, _, follows = CAMERAS[self.cam_idx]
        _, _, heading = self.bot.pose()
        base_az = np.degrees(heading) + 180 if follows else 90
        self.cam.azimuth = base_az + self.az_offset
        self.cam.elevation = self.el
        self.cam.distance = self.dist
        self.cam.lookat[:] = self.data.body("chassis").xpos + self.pan

        self.renderer.update_scene(self.data, self.cam)
        # frombuffer avoids a full array transpose; make_surface is ~9x slower.
        frame = pygame.image.frombuffer(
            self.renderer.render().tobytes(), (WIN_W, WIN_H), "RGB")
        self.screen.blit(frame, (0, 0))
        self.hud(name)
        pygame.display.flip()

    def hud(self, cam_name):
        x, y, h = self.bot.pose()
        arm = float(self.data.joint("shoulder").qpos[0])
        panel = pygame.Surface((252, 208), pygame.SRCALPHA)
        panel.fill((12, 14, 18, 195))
        self.screen.blit(panel, (14, 14))

        mode = "FIELD-CENTRIC" if self.field_centric else "ROBOT-CENTRIC"
        rows = [
            (self.big, mode, (110, 200, 255) if self.field_centric else (200, 205, 212)),
            (self.font, f"x        {x:+.2f} m", (222, 226, 232)),
            (self.font, f"y        {y:+.2f} m", (222, 226, 232)),
            (self.font, f"heading  {np.degrees(h):+7.1f} deg", (222, 226, 232)),
            (self.font, f"arm      {arm:+.2f} rad", (222, 226, 232)),
            (self.font, f"claw     {'CLOSED' if self.grip > 0.5 else 'open'}",
             (255, 165, 60) if self.grip > 0.5 else (222, 226, 232)),
            (self.font, f"hold     {'ON' if self.heading_hold else 'off'}",
             (110, 220, 140) if self.heading_hold else (150, 155, 165)),
            (self.font, f"camera   {cam_name}"
                        f"{'  (panned)' if np.linalg.norm(self.pan) > 0.01 else ''}",
             (255, 200, 110) if np.linalg.norm(self.pan) > 0.01 else (150, 155, 165)),
            (self.font, f"{self.fps:.0f} fps"
                        f"{'  [pad]' if self.js else ''}", (120, 126, 136)),
        ]
        yy = 24
        for font, text, colour in rows:
            self.screen.blit(font.render(text, True, colour), (26, yy))
            yy += 21

        hint = ("WASD drive  QE turn  RF arm  SPACE claw  G field  H hold  |  "
                "L-drag orbit  R-drag pan  scroll zoom  TAB cam  HOME recentre")
        surf = self.font.render(hint, True, (150, 156, 166))
        bg = pygame.Surface((surf.get_width() + 24, 30), pygame.SRCALPHA)
        bg.fill((12, 14, 18, 195))
        self.screen.blit(bg, (14, WIN_H - 44))
        self.screen.blit(surf, (26, WIN_H - 37))

    # -- main loop ------------------------------------------------------

    def run(self):
        dt = self.model.opt.timestep
        acc = 0.0
        last = time.perf_counter()
        smooth = 0.0

        while self.running:
            now = time.perf_counter()
            elapsed = now - last
            last = now
            acc += min(elapsed, 0.25)          # cap so a stall can't spiral
            smooth = 0.9 * smooth + 0.1 * elapsed
            self.fps = 1.0 / smooth if smooth > 0 else 0.0

            self.handle_events()
            s = self.sticks()

            # Run however many fixed physics steps fit the real time elapsed, so
            # holding a key produces continuous motion at true 1x speed.
            steps = 0
            while acc >= dt and steps < 200:
                self.step_physics(s, 1)
                acc -= dt
                steps += 1

            self.draw()

            slack = FRAME - (time.perf_counter() - now)
            if slack > 0:
                time.sleep(slack)

        pygame.quit()


if __name__ == "__main__":
    print("ftcsim teleop - single window, all input owned by us.")
    print("WASD drive, QE turn, RF arm, SPACE claw, G field-centric, TAB camera.")
    Sim().run()
