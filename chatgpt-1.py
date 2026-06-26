# main.py — Mecanum 4-Stepper Robot Tango Skeleton (MicroPython)

import time
from machine import Pin

# ------------------------------------------------------------
# Stepper Driver (unchanged core)
# ------------------------------------------------------------

class Stepper:
    SEQ = [
        (1, 0, 0, 0),
        (1, 1, 0, 0),
        (0, 1, 0, 0),
        (0, 1, 1, 0),
        (0, 0, 1, 0),
        (0, 0, 1, 1),
        (0, 0, 0, 1),
        (1, 0, 0, 1),
    ]

    def __init__(self, p1, p2, p3, p4):
        self.pins = [
            Pin(p1, Pin.OUT),
            Pin(p2, Pin.OUT),
            Pin(p3, Pin.OUT),
            Pin(p4, Pin.OUT),
        ]

        self.index = 0
        self.speed = 0  # steps/sec signed
        self.last = time.ticks_ms()

    def _apply(self, pattern):
        for pin, val in zip(self.pins, pattern):
            pin.value(val)

    def stop(self):
        self.speed = 0
        for p in self.pins:
            p.value(0)

    def set_speed(self, speed):
        self.speed = speed

    def update(self):
        if self.speed == 0:
            return

        now = time.ticks_ms()
        interval = int(1000 / abs(self.speed))

        if time.ticks_diff(now, self.last) < interval:
            return

        self.last = now

        if self.speed > 0:
            self.index = (self.index + 1) % 4
        else:
            self.index = (self.index - 1) % 4

        self._apply(self.SEQ[self.index])


# ------------------------------------------------------------
# Robot: 4-wheel Mecanum
# ------------------------------------------------------------

class Robot:
    def __init__(self):
        self.fl = Stepper(8, 9, 10, 11)
        self.fr = Stepper(20, 21, 22, 7)
        self.rl = Stepper(12, 13, 14, 15)
        self.rr = Stepper(16, 17, 18, 19)

        # target wheel speeds (from kinematics)
        self.w_fl = 0
        self.w_fr = 0
        self.w_rl = 0
        self.w_rr = 0

    def stop(self):
        self.set_wheels(0, 0, 0, 0)

    def set_wheels(self, fl, fr, rl, rr):
        self.w_fl = fl
        self.w_fr = fr
        self.w_rl = rl
        self.w_rr = rr

        self.fl.set_speed(fl)
        self.fr.set_speed(fr)
        self.rl.set_speed(rl)
        self.rr.set_speed(rr)

    # --------------------------------------------------------
    # KINEMATICS-FIRST API
    # --------------------------------------------------------

    def set_velocity(self, vx, vy, omega):
        """
        vx     : forward (+) / backward (-)
        vy     : right (+) / left (-)
        omega  : rotation CW (+)
        """

        fl = vx - vy - omega
        fr = vx + vy + omega
        rl = vx + vy - omega
        rr = vx - vy + omega

        # normalize so no wheel exceeds limit
        m = max(abs(fl), abs(fr), abs(rl), abs(rr), 1)

        scale = 1.0 if m < 1 else (1.0 / m)

        self.set_wheels(
            int(fl * scale),
            int(fr * scale),
            int(rl * scale),
            int(rr * scale),
        )

    def update(self):
        self.fl.update()
        self.fr.update()
        self.rl.update()
        self.rr.update()

# ------------------------------------------------------------
# Ramp Controller (now 4-channel)
# ------------------------------------------------------------

class RampController:
    def __init__(self, robot, ramp_ms=400):
        self.robot = robot
        self.ramp_ms = ramp_ms

        self.cur = [0.0, 0.0, 0.0]  # vx, vy, omega
        self.tgt = [0.0, 0.0, 0.0]

        self.last = time.ticks_ms()

    def set_target(self, vx, vy, omega):
        self.tgt = [vx, vy, omega]

    def update(self):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last)
        self.last = now

        step = dt / self.ramp_ms if self.ramp_ms else 1.0
        if step > 1:
            step = 1

        for i in range(3):
            self.cur[i] += (self.tgt[i] - self.cur[i]) * step

        self.robot.set_velocity(
            self.cur[0],
            self.cur[1],
            self.cur[2],
        )

# ------------------------------------------------------------
# Scheduler (unchanged)
# ------------------------------------------------------------

class Scheduler:
    def __init__(self):
        self.events = []
        self.t0 = None
        self.i = 0

    def add(self, t, action, *args):
        self.events.append((t, action, args))
        self.events.sort()

    def start(self):
        self.t0 = time.ticks_ms()
        self.i = 0

    def update(self, ctx):
        if self.t0 is None:
            return

        elapsed = time.ticks_diff(time.ticks_ms(), self.t0) / 1000

        while self.i < len(self.events):
            t, action, args = self.events[self.i]

            if elapsed < t:
                break

            getattr(ctx, action)(*args)
            self.i += 1


# ------------------------------------------------------------
# Choreography (Part 1)
# ------------------------------------------------------------

class Choreography:
    def __init__(self, robot, ramp):
        self.robot = robot
        self.ramp = ramp
        self.scheduler = Scheduler()
        self.build()

    def build(self):
        # 5s intro (no movement)
        self.scheduler.add(5.0, "forward", 120)
        self.scheduler.add(6.0, "strafe_right", 120)
        self.scheduler.add(7.0, "rotate", 120)
        self.scheduler.add(8.0, "strafe_left", 120)
        self.scheduler.add(9.0, "stop")

    # ramped actions
    def forward(self, s):
        self.ramp.set_target(s, 0, 0)

    def backward(self, s):
        self.ramp.set_target(-s, 0, 0)

    def strafe_right(self, s):
        self.ramp.set_target(0, s, 0)

    def strafe_left(self, s):
        self.ramp.set_target(0, -s, 0)

    def rotate(self, s):
        self.ramp.set_target(0, 0, s)

    def stop(self):
        self.ramp.set_target(0, 0, 0, 0)

    def start(self):
        self.scheduler.start()

    def update(self):
        self.scheduler.update(self)
        self.ramp.update()
        self.robot.update()


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

robot = Robot()
ramp = RampController(robot, ramp_ms=500)
choreo = Choreography(robot, ramp)

choreo.start()

while True:
    choreo.update()
    time.sleep_ms(10)