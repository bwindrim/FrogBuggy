# main.py — Mecanum 4-Stepper Robot Tango Skeleton (MicroPython)

import time
from machine import Pin

import math

MAX_STEPS = 1200

class Keyframe:
    def __init__(self, t, x, y, theta):
        self.t = t
        self.x = x
        self.y = y
        self.theta = theta

def smoothstep(u):
    return u * u * (3 - 2 * u)


def lerp(a, b, u):
    return a + (b - a) * u
class Trajectory:
    def __init__(self, keyframes):
        self.kf = sorted(keyframes, key=lambda k: k.t)

    def sample(self, t):
        # clamp edges
        if t <= self.kf[0].t:
            k = self.kf[0]
            return k.x, k.y, k.theta

        if t >= self.kf[-1].t:
            k = self.kf[-1]
            return k.x, k.y, k.theta

        # find segment
        for i in range(len(self.kf) - 1):
            a = self.kf[i]
            b = self.kf[i + 1]

            if a.t <= t <= b.t:
                u = (t - a.t) / (b.t - a.t)
                u = smoothstep(u)

                x = lerp(a.x, b.x, u)
                y = lerp(a.y, b.y, u)
                th = lerp(a.theta, b.theta, u)

                return x, y, th

class PoseController:
    def __init__(self):
        self.last = None

    def compute_velocity(self, x, y, theta, dt):
        """
        Very simple derivative-based controller.
        (good enough for choreography / dance robot)
        """

        if self.last is None:
            self.last = (x, y, theta)
            return 0, 0, 0

        lx, ly, lth = self.last

        vx = (x - lx) / dt
        vy = (y - ly) / dt
        omega = (theta - lth) / dt

        self.last = (x, y, theta)

        return vx, vy, omega

class TrajectoryRunner:
    def __init__(self, robot, traj):
        self.robot = robot
        self.traj = traj
        self.ctrl = PoseController()

        self.t0 = time.ticks_ms()

    def update(self):
        now = time.ticks_ms()
        t = time.ticks_diff(now, self.t0) / 1000.0

        x, y, th = self.traj.sample(t)

        vx, vy, omega = self.ctrl.compute_velocity(x, y, th, 0.05)

        self.robot.set_velocity(vx * 50, vy * 50, omega * 20)



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
            self.index = (self.index + 1) % 8
        else:
            self.index = (self.index - 1) % 8

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

        GAIN = 200  # <- important tuning constant

        fl *= GAIN
        fr *= GAIN
        rl *= GAIN
        rr *= GAIN

        m = max(abs(fl), abs(fr), abs(rl), abs(rr), 1)

        if m > MAX_STEPS:
            scale = MAX_STEPS / m
        else:
            scale = 1.0

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

keyframes = [
    Keyframe(0.0,  0, 0, 0),
    Keyframe(1.5,  1, 0, 0),
    Keyframe(3.0,  1, 1, 1.57),
    Keyframe(4.5,  0, 1, 3.14),
    Keyframe(6.0,  0, 0, 0),
]


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

robot = Robot()

traj = Trajectory(keyframes)
runner = TrajectoryRunner(robot, traj)

try:
    while True:
        robot.set_velocity(1, 0, 0)
        robot.update()
        time.sleep_ms(10)
except KeyboardInterrupt:
    print("Interrupted by user")
except Exception as e:
    print("Error:", e)
finally:
    robot.stop()
    print("Motors released")
