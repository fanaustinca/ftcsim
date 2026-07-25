"""Drive the robot yourself.

    python teleop.py

Two windows open: the 3D view, and a small control panel. CLICK THE CONTROL
PANEL so it has keyboard focus, then drive.

    W / S      forward / back          SPACE  toggle claw
    A / D      strafe left / right     G      toggle field-centric
    Q / E      turn left / right       BKSP   reset
    R / F      arm up / down           ESC    quit

Plug in an Xbox-style controller before launching and it takes over: left stick
drives, right stick X turns, triggers work the arm, A/B the claw.

WHY THE SEPARATE WINDOW: MuJoCo's viewer binds every letter A-Z to a render
flag -- W is wireframe, S is shadow, R is reflection, and so on. A key_callback
runs in ADDITION to those, so driving with WASD in the viewer window also
strobes the render settings. Owning the keyboard in our own window is the only
clean fix, and it gets us real held-key state as a bonus.
"""
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np
import pygame

from robot import build_mjcf
from motors import Robot

DEADZONE = 0.08
PANEL_W, PANEL_H = 460, 296


def deadzone(v):
    return 0.0 if abs(v) < DEADZONE else (abs(v) - DEADZONE) / (1 - DEADZONE) * np.sign(v)


class Input:
    """Keyboard (via our own pygame window) with gamepad override."""

    def __init__(self):
        pygame.init()
        pygame.display.set_caption("FTC Sim - click here, then drive")
        self.screen = pygame.display.set_mode((PANEL_W, PANEL_H))
        self.font = pygame.font.SysFont("monospace", 15)
        self.big = pygame.font.SysFont("monospace", 17, bold=True)

        pygame.joystick.init()
        self.js = None
        if pygame.joystick.get_count():
            self.js = pygame.joystick.Joystick(0)
            self.js.init()

        self.grip_closed = False
        self.field_centric = False
        self.quit = False
        self.reset = False

    def poll(self):
        self.reset = False
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                self.quit = True
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_SPACE:
                    self.grip_closed = not self.grip_closed
                elif e.key == pygame.K_g:
                    self.field_centric = not self.field_centric
                elif e.key == pygame.K_BACKSPACE:
                    self.reset = True
                elif e.key == pygame.K_ESCAPE:
                    self.quit = True
            elif e.type == pygame.JOYBUTTONDOWN and self.js:
                if e.button == 0:
                    self.grip_closed = True
                elif e.button == 1:
                    self.grip_closed = False
                elif e.button == 2:
                    self.field_centric = not self.field_centric
                elif e.button == 6:
                    self.reset = True

        # Keyboard: get_pressed() is real held state, so holding a key means
        # "keep going" rather than one impulse per press.
        k = pygame.key.get_pressed()
        axis = lambda a, b: float(k[a]) - float(k[b])
        out = dict(
            fwd=axis(pygame.K_w, pygame.K_s),
            strafe=axis(pygame.K_d, pygame.K_a),
            turn=axis(pygame.K_e, pygame.K_q),
            arm=axis(pygame.K_r, pygame.K_f),
        )

        # A connected gamepad ADDS to the keyboard rather than replacing it.
        # Returning early on self.js meant a controller that was plugged in but
        # idle silently swallowed WASD -- and in WSL a phantom HID device is
        # enough to trigger that.
        if self.js:
            ax = lambda i: deadzone(self.js.get_axis(i)) if self.js.get_numaxes() > i else 0.0
            lt = (self.js.get_axis(4) + 1) / 2 if self.js.get_numaxes() > 4 else 0.0
            rt = (self.js.get_axis(5) + 1) / 2 if self.js.get_numaxes() > 5 else 0.0
            for key, v in dict(fwd=-ax(1), strafe=ax(0), turn=ax(3),
                               arm=rt - lt).items():
                if abs(v) > abs(out[key]):
                    out[key] = v
        return out

    @property
    def focused(self):
        """Does this window have keyboard focus? If not, WASD goes nowhere."""
        return bool(pygame.key.get_focused())

    def draw(self, bot, grip, arm_angle):
        # Red background when unfocused: the single most common "it doesn't
        # work" cause is typing into the 3D window instead of this one.
        self.screen.fill((22, 24, 30) if self.focused else (74, 26, 26))
        x, y, h = bot.pose()
        src = self.js.get_name()[:26] if self.js else "keyboard (click this window)"

        rows = [
            (self.big,
             "READY - drive it" if self.focused else "CLICK HERE FIRST",
             (110, 220, 140) if self.focused else (255, 190, 90)),
            (self.big, f"{'FIELD-CENTRIC' if self.field_centric else 'ROBOT-CENTRIC'}",
             (110, 200, 255) if self.field_centric else (170, 175, 185)),
            (self.font, f"input   {src}", (140, 145, 155)),
            (self.font, "", None),
            (self.font, f"x       {x:+.2f} m", (225, 228, 235)),
            (self.font, f"y       {y:+.2f} m", (225, 228, 235)),
            (self.font, f"heading {np.degrees(h):+7.1f} deg", (225, 228, 235)),
            (self.font, f"arm     {arm_angle:+.2f} rad", (225, 228, 235)),
            (self.font, f"claw    {'CLOSED' if grip > 0.5 else 'open'}",
             (255, 160, 60) if grip > 0.5 else (225, 228, 235)),
            (self.font, "", None),
            (self.font, "WASD drive  QE turn  RF arm", (120, 125, 135)),
            (self.font, "SPACE claw  G field  BKSP reset", (120, 125, 135)),
        ]
        yy = 14
        for font, text, colour in rows:
            if text:
                self.screen.blit(font.render(text, True, colour or (200, 200, 200)), (18, yy))
            yy += 21
        pygame.display.flip()


def main():
    model = mujoco.MjModel.from_xml_string(build_mjcf())
    data = mujoco.MjData(model)
    bot = Robot(model, data)
    bot.stow()

    inp = Input()
    grip = 0.0
    arm_target = 1.2
    manual_arm = False
    last_draw = 0.0

    print("Two windows opened. Click the CONTROL PANEL to drive.")
    print("(Keys typed into the 3D window toggle MuJoCo's render flags instead.)")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 2.0
        viewer.cam.elevation = -25

        # Physics and rendering are decoupled. The old loop did one mj_step, one
        # viewer.sync and one time.sleep(1.94 ms) per iteration, 500 times a
        # second. Sleeping at that granularity is unreliable, and pygame's
        # display.flip() can vsync-block for a whole 16 ms frame -- so the sim
        # advanced in lurches, which is why holding a key felt like discrete
        # nudges rather than continuous motion.
        #
        # Now: measure real elapsed time, run however many 2 ms physics steps
        # fit inside it, then draw once. The command is held across every step
        # in the batch, so holding a key really does mean "keep going".
        FRAME = 1.0 / 60.0
        dt_phys = model.opt.timestep
        accumulator = 0.0
        last = time.perf_counter()

        while viewer.is_running() and not inp.quit:
            now = time.perf_counter()
            accumulator += min(now - last, 0.25)   # cap: a stall can't spiral
            last = now

            s = inp.poll()

            if inp.reset:
                mujoco.mj_resetData(model, data)
                bot.stow()
                arm_target, grip = 1.2, 0.0
                accumulator = 0.0

            while accumulator >= dt_phys:
                fwd, strafe = s["fwd"], s["strafe"]
                if inp.field_centric:
                    h = bot.heading()
                    fwd, strafe = (fwd * np.cos(-h) - strafe * np.sin(-h),
                                   fwd * np.sin(-h) + strafe * np.cos(-h))
                bot.drive(fwd, strafe, s["turn"])

                # Hold wherever the arm was left when the key/trigger is released.
                if abs(s["arm"]) > 0.05:
                    manual_arm = True
                    bot.set_arm(s["arm"])
                else:
                    if manual_arm:
                        arm_target = float(data.joint("shoulder").qpos[0])
                        manual_arm = False
                    bot.hold_arm(arm_target)
                bot.level_wrist(0.0)

                grip = (min(1.0, grip + 0.02) if inp.grip_closed
                        else max(0.0, grip - 0.02))
                bot.set_grip(grip)

                bot.apply()
                mujoco.mj_step(model, data)
                accumulator -= dt_phys

            viewer.cam.lookat[:] = data.body("chassis").xpos
            viewer.sync()
            inp.draw(bot, grip, float(data.joint("shoulder").qpos[0]))

            slack = FRAME - (time.perf_counter() - now)
            if slack > 0:
                time.sleep(slack)

    pygame.quit()


if __name__ == "__main__":
    main()
