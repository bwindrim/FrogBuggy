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

class DanceBuilder:
    def __init__(self):
        self.t = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.frames = [Keyframe(0.0, 0.0, 0.0, 0.0)]

    def _add(self, dt):
        self.t += dt
        self.frames.append(
            Keyframe(self.t, self.x, self.y, self.theta)
        )
        return self

    def hold(self, dt):
        return self._add(dt)

    def forward(self, distance, dt):
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)
        return self._add(dt)

    def backward(self, distance, dt):
        return self.forward(-distance, dt)

    def left(self, distance, dt):
        self.x += -distance * math.sin(self.theta)
        self.y +=  distance * math.cos(self.theta)
        return self._add(dt)

    def right(self, distance, dt):
        return self.left(-distance, dt)

    def turn_left(self, degrees, dt):
        self.theta = angle_wrap(
            self.theta + math.radians(degrees)
        )
        return self._add(dt)

    def turn_right(self, degrees, dt):
        return self.turn_left(-degrees, dt)

    def build(self):
        return self.frames

    def wait_for_music(self):
        return self.hold(5.0)

    def sway(self, distance=0.25, beat=0.5):
        return (self
            .left(distance, beat)
            .right(distance * 2, beat)
            .left(distance, beat))

    def box_step(self, size=0.5, beat=0.7):
        return (self
            .forward(size, beat)
            .right(size, beat)
            .backward(size, beat)
            .left(size, beat))

    def promenade(self, distance=1.0, duration=2.0):
        return self.forward(distance, duration)

    def quarter_turn_left(self, beat=0.8):
        return self.turn_left(90, beat)

    def quarter_turn_right(self, beat=0.8):
        return self.turn_right(90, beat)

    def half_turn(self, beat=1.5):
        return self.turn_left(180, beat)

    def spin_left(self, turns=1.0, duration=2.0):
        return self.turn_left(360 * turns, duration)

    def spin_right(self, turns=1.0, duration=2.0):
        return self.turn_right(360 * turns, duration)

    def pause(self, beat=0.5):
        return self.hold(beat)

    def glide(self, distance, dt):
        return self.left(distance, dt)

    def arc_left(self,
                radius,
                degrees,
                duration,
                segments=8):

        angle = math.radians(degrees)

        cx = self.x - radius * math.sin(self.theta)
        cy = self.y + radius * math.cos(self.theta)

        start = self.theta

        for i in range(1, segments + 1):

            a = start + angle * i / segments

            self.x = cx + radius * math.sin(a)
            self.y = cy - radius * math.cos(a)
            self.theta = angle_wrap(a)

            self._add(duration / segments)

        return self

    def arc_right(self,
                radius,
                degrees,
                duration,
                segments=8):

        angle = math.radians(degrees)

        cx = self.x + radius * math.sin(self.theta)
        cy = self.y - radius * math.cos(self.theta)

        start = self.theta

        for i in range(1, segments + 1):

            a = start - angle * i / segments

            self.x = cx - radius * math.sin(a)
            self.y = cy + radius * math.cos(a)
            self.theta = angle_wrap(a)

            self._add(duration / segments)

        return self

    def dip(self, amount=0.2, beat=0.25):
        return (
            self
            .backward(amount, beat)
            .forward(amount, beat)
        )

def smoothstep(u: float) -> float:
    return u * u * (3 - 2 * u)

def smoothstep_derivative(u: float) -> float:
    # derivative of u²(3−2u)
    return 6.0 * u * (1.0 - u)

def lerp(a: float, b: float, u: float) -> float:
    return a + (b - a) * u

def angle_wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def angle_lerp(a: float, b: float, u: float) -> float:
    d = angle_wrap(b - a)
    return angle_wrap(a + d * u)

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
                th = angle_lerp(a.theta, b.theta, u)

                vx = (b.x - a.x) * du / duration
                vy = (b.y - a.y) * du / duration
                dtheta = angle_wrap(b.theta - a.theta)
                omega = dtheta * du / duration

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
        self.target_speed = 0.0      # requested speed (steps/s)
        self.current_speed = 0.0     # actual speed (steps/s)

        self.max_accel = 2500.0      # steps/s² (tune this)

        self.next_step = time.ticks_us()
        self.last_update = time.ticks_us()
        self.next_step = time.ticks_us()

    def _apply(self, pattern) -> None:
        for pin, val in zip(self.pins, pattern):
            pin.value(val)

    def stop(self) -> None:
        self.target_speed = 0.0
        self.current_speed = 0.0
        for p in self.pins:
            p.value(0)

    def set_speed(self, speed: int) -> None:
        self.target_speed = float(speed)

    def update(self) -> None:
        now = time.ticks_us()

        dt = time.ticks_diff(now, self.last_update) / 1_000_000.0
        self.last_update = now

        delta = self.target_speed - self.current_speed

        limit = self.max_accel * dt

        if delta > limit:
            delta = limit
        elif delta < -limit:
            delta = -limit

        self.current_speed += delta

        # only stop AFTER we've updated the speed
        if abs(self.current_speed) < 0.01:
            return

        interval = max(100, int(1_000_000 / abs(self.current_speed)))

        now = time.ticks_us()

        if time.ticks_diff(now, self.next_step) >= 0:
            self._step_once()
            self.next_step = time.ticks_add(self.next_step, interval)

            # If we've fallen a long way behind (e.g. after a pause),
            # resynchronise rather than trying to catch up with a burst.
            if time.ticks_diff(now, self.next_step) > interval * 4:
                self.next_step = time.ticks_add(now, interval)
    
    def _step_once(self):
        if self.current_speed > 0:
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



keyframes: list = (
    DanceBuilder()
#        .hold(5.0)          # spoken introduction
        .forward(1.0, 1.5)
        .left(0.8, 1.2)
        .right(0.8, 1.2)
        .turn_left(90, 1.5)
        .forward(1.0, 1.5)
        .turn_left(90, 1.5)
        .forward(1.0, 1.5)
        .turn_left(90, 1.5)
        .forward(1.0, 1.5)
        .build()
)
keyframes = (
    DanceBuilder()
#        .wait_for_music()
        .promenade(1.0, 1.5)
        .sway()
        .quarter_turn_left()
        .promenade(1.0, 1.5)
        .quarter_turn_left()
        .promenade(1.0, 1.5)
        .quarter_turn_left()
        .promenade(1.0, 1.5)
        .build()
)
keyframes = (
    DanceBuilder()
        .wait_for_music()
        .promenade(0.8,1.2)
        .sway()
        .arc_left(0.7,90,2.0)
        .glide(0.5,0.7)
        .glide(-0.5,0.7)
        .spin_left(1.0,2.0)
        .dip()
        .pause(0.5)
        .build()
)
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
