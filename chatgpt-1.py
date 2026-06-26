# main.py — Mecanum 4-Stepper Robot Tango Skeleton (MicroPython)

import time
from machine import Pin

import math

MAX_STEPS: int = 300

class Keyframe:
    def __init__(self, t: float, x: float, y: float, theta: float) -> None:
        self.t: float = t
        self.x: float = x
        self.y: float = y
        self.theta: float = theta

def smoothstep(u: float) -> float:
    return u * u * (3 - 2 * u)

def smoothstep_derivative(u: float) -> float:
    # derivative of u²(3−2u)
    return 6.0 * u * (1.0 - u)

def lerp(a: float, b: float, u: float) -> float:
    return a + (b - a) * u

class Trajectory:
    def __init__(self, keyframes: list) -> None:
        self.kf: list = sorted(keyframes, key=lambda k: k.t)

    def sample(self, t: float):

        if t <= self.kf[0].t:
            k = self.kf[0]
            return k.x, k.y, k.theta, 0.0, 0.0, 0.0

        if t >= self.kf[-1].t:
            k = self.kf[-1]
            return k.x, k.y, k.theta, 0.0, 0.0, 0.0

        for i in range(len(self.kf) - 1):

            a = self.kf[i]
            b = self.kf[i + 1]

            if a.t <= t <= b.t:

                duration = b.t - a.t

                s = (t - a.t) / duration

                u = smoothstep(s)
                du = smoothstep_derivative(s)

                x = lerp(a.x, b.x, u)
                y = lerp(a.y, b.y, u)
                th = lerp(a.theta, b.theta, u)

                vx = (b.x - a.x) * du / duration
                vy = (b.y - a.y) * du / duration
                omega = (b.theta - a.theta) * du / duration

                return x, y, th, vx, vy, omega


class TrajectoryRunner:
    def __init__(self, robot, traj) -> None:
        self.robot = robot
        self.traj = traj
        self.t0 = time.ticks_us()

    def update(self):

        now = time.ticks_us()
        t = time.ticks_diff(now, self.t0) / 1_000_000.0

        x, y, th, vx, vy, omega = self.traj.sample(t)

        # convert trajectory units to wheel-speed units
        self.robot.set_velocity(
            vx * 50,
            vy * 50,
            omega * 20,
        )

# ------------------------------------------------------------
# Stepper Driver (unchanged core)
# ------------------------------------------------------------

class Stepper:
    SEQ: list = [
        (1, 0, 0, 0),
        (1, 1, 0, 0),
        (0, 1, 0, 0),
        (0, 1, 1, 0),
        (0, 0, 1, 0),
        (0, 0, 1, 1),
        (0, 0, 0, 1),
        (1, 0, 0, 1),
    ]

    def __init__(self, p1: int, p2: int, p3: int, p4: int) -> None:
        self.pins: list = [
            Pin(p1, Pin.OUT),
            Pin(p2, Pin.OUT),
            Pin(p3, Pin.OUT),
            Pin(p4, Pin.OUT),
        ]

        self.index: int = 0
        self.speed: int = 0  # steps/sec signed
        self.next_step = time.ticks_us()

    def _apply(self, pattern) -> None:
        for pin, val in zip(self.pins, pattern):
            pin.value(val)

    def stop(self) -> None:
        self.speed = 0
        for p in self.pins:
            p.value(0)

    def set_speed(self, speed: int) -> None:
        if speed != self.speed:
            self.speed = speed
            self.next_step = time.ticks_us()

    def update(self) -> None:
        if self.speed == 0:
            return

        interval = max(100, int(1_000_000 / abs(self.speed)))

        now = time.ticks_us()

        if time.ticks_diff(now, self.next_step) >= 0:
            self._step_once()
            self.next_step = time.ticks_add(now, interval)

            # If we've fallen a long way behind (e.g. after a pause),
            # resynchronise rather than trying to catch up with a burst.
            if time.ticks_diff(now, self.next_step) > interval * 4:
                self.next_step = time.ticks_add(now, interval)
    
    def _step_once(self):
        if self.speed > 0:
            self.index = (self.index + 1) % len(self.SEQ)
        else:
            self.index = (self.index - 1) % len(self.SEQ)

        self._apply(self.SEQ[self.index])

# ------------------------------------------------------------
# Robot: 4-wheel Mecanum
# ------------------------------------------------------------

class Robot:
    def __init__(self) -> None:
        self.fl = Stepper(8, 9, 10, 11)
        self.fr = Stepper(20, 21, 22, 7)
        self.rl = Stepper(12, 13, 14, 15)
        self.rr = Stepper(16, 17, 18, 19)

        # target wheel speeds (from kinematics)
        self.w_fl: int = 0
        self.w_fr: int = 0
        self.w_rl: int = 0
        self.w_rr: int = 0

    def stop(self) -> None:
        self.set_wheels(0, 0, 0, 0)

    def set_wheels(self, fl: int, fr: int, rl: int, rr: int) -> None:
        print(fl, fr, rl, rr)
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

    def set_velocity(self, vx: float, vy: float, omega: float) -> None:
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

    def update(self) -> None:
        self.fl.update()
        self.fr.update()
        self.rl.update()
        self.rr.update()



keyframes: list = [
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
    print("Starting.")
    while True:
        runner.update()
        robot.update()
        time.sleep_ms(1)
except KeyboardInterrupt:
    print("Interrupted by user")
except Exception as e:
    print("Error:", e)
finally:
    robot.stop()
    print("Motors released")
