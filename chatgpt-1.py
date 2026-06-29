# main.py — Mecanum 4-Stepper Robot Tango Skeleton (MicroPython)

from machine import Pin
from rp2 import bootsel_button
import time
from time import sleep
from dfplayermini import DFPlayerMini
import math

led = Pin('LED', Pin.OUT)
hand = Pin(26, Pin. IN, Pin.PULL_UP)

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

    def wait_for_music(self, wait_time=5.0):
        return self.hold(wait_time)

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

    def forward_left(self, distance, dt):
        d = distance / math.sqrt(2.0)
        self.x += d * math.cos(self.theta) - d * math.sin(self.theta)
        self.y += d * math.sin(self.theta) + d * math.cos(self.theta)
        return self._add(dt)

    def forward_right(self, distance, dt):
        d = distance / math.sqrt(2.0)
        self.x += d * math.cos(self.theta) + d * math.sin(self.theta)
        self.y += d * math.sin(self.theta) - d * math.cos(self.theta)
        return self._add(dt)

    def backward_left(self, distance, dt):
        return self.forward_right(-distance, dt)

    def backward_right(self, distance, dt):
        return self.forward_left(-distance, dt)

    def corte(self, beat=0.30):
        """Sharp tango freeze."""
        return self.hold(beat)

    def figure_eight(self,
                     radius=0.6,
                     duration=4.0,
                     segments=8):
        return (
            self
            .arc_left(
                radius,
                180,
                duration / 2,
                segments
            )
            .arc_right(
                radius,
                180,
                duration / 2,
                segments
            )
        )

    def reverse_figure_eight(self,
                             radius=0.6,
                             duration=4.0,
                             segments=8):
        return (
            self
            .arc_right(
                radius,
                180,
                duration / 2,
                segments
            )
            .arc_left(
                radius,
                180,
                duration / 2,
                segments
            )
        )

    def spiral(self,
               turns=1.0,
               start_radius=1.0,
               end_radius=0.2,
               duration=5.0,
               segments=24,
               direction="left"):

        total_angle = 360.0 * turns

        for i in range(segments):

            u0 = i / segments
            u1 = (i + 1) / segments

            radius = (
                start_radius +
                (end_radius - start_radius) * u0
            )

            if direction == "left":
                self.arc_left(
                    radius=radius,
                    degrees=total_angle / segments,
                    duration=duration / segments,
                    segments=1
                )
            else:
                self.arc_right(
                    radius=radius,
                    degrees=total_angle / segments,
                    duration=duration / segments,
                    segments=1
                )

        return self

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
    def __init__(self, robot, traj):
        self.robot = robot
        self.traj = traj
        self.t0 = None
        self.duration = traj.kf[-1].t

    def update(self):
        if self.t0 is None:
            self.t0 = time.ticks_us()
        now = time.ticks_us()
        t = time.ticks_diff(now, self.t0) / 1_000_000.0

        if t >= self.duration:
            self.robot.set_velocity(0.0, 0.0, 0.0)
            return False

        x, y, th, vx, vy, omega = self.traj.sample(t)

        self.robot.set_velocity(
            vx * 50,
            vy * 50,
            omega * 20,
        )

        return True

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

def intro(db):
    return (db
        .wait_for_music(wait_time=7.5)
        .sway(distance=0.08, beat=0.45)
        .sway(distance=0.06, beat=0.40)
        .turn_left(degrees=20, dt=1.4)
        .forward_left(distance=0.25, dt=1.4)
        .corte(beat=0.40)
    )

def opening_A(db):
    return (db
        # Strong entrance
        .promenade(distance=0.90, duration=3.0)
        # Sweep gently onto the dance floor
        .arc_left(
            radius=0.90,
            degrees=70,
            duration=3.0)
        # Continue confidently
        .promenade(distance=0.75, duration=2.5)
        # Small hesitation
        # Elegant tango sway
        .sway(
            distance=0.15,
            beat=0.50)
        # Finish by opening the body slightly
        .turn_left(
            degrees=20,
            dt=0.8)
        .corte(beat=0.30)
    )

def opening_A_prime(db=db):
    return (db

        # Repeat the opening idea, but mirrored.
        .promenade(distance=0.95, duration=3.2)
        # Mirror of the previous sweeping turn.
        .arc_right(
            radius=0.90,
            degrees=70,
            duration=3.0)
        # Continue travelling.
        .promenade(distance=0.85, duration=2.8)
        # Diagonal glide.
        .forward_right(distance=0.35, dt=1.6)
        # Recover gracefully.
        .backward_left(distance=0.18, dt=0.9)
        # Small flourish.
        .turn_right(
            degrees=18,
            dt=0.8)
        .corte(beat=0.35)
    )

def bridge(db=db):
    return (db

        # Large flowing figure that uses the floor.
        .figure_eight(
            radius=0.80,
            duration=6.0)
        # Travel confidently.
        .promenade(
            distance=1.10,
            duration=3.5)
        # Gentle side conversation.
        .glide(distance=0.25, dt=0.8)
        .glide(distance=-0.25, dt=0.8)
        # Open the couple towards the centre.
        .arc_left(
            radius=0.75,
            degrees=90,
            duration=2.8)
        .corte(beat=0.40)
    )

def development_A(db):
    return (db

        # A confident diagonal entrance into the phrase.
        .forward_left(distance=0.70, dt=2.6)
        # Mirror immediately.
        .forward_right(distance=0.70, dt=2.6)
        # Flow into a broad right-hand curve.
        .arc_right(
            radius=1.00,
            degrees=120,
            duration=3.8)
        # Continue travelling.
        .promenade(distance=0.90, duration=2.8)
        # Gentle weight shift.
        .glide(distance=0.20, dt=0.7)
        .glide(distance=-0.20, dt=0.7)
        .corte(beat=0.30)
    )

def development_B(db):
    return (db

        # Long sweeping curve across the floor.
        .arc_left(
            radius=1.20,
            degrees=160,
            duration=5.0)
        # Continue without stopping.
        .promenade(
            distance=1.10,
            duration=3.2)
        # Tightening spiral.
        .spiral(
            turns=0.5,
            start_radius=1.00,
            end_radius=0.45,
            duration=3.8,
            segments=12)
        # Drift diagonally.
        .forward_left(distance=0.60, dt=2.2)
        # Gentle recovery.
        .backward_right(distance=0.30, dt=1.2)
        # Finally acknowledge the musical cadence.
        .corte(beat=0.35)
    )

def development_C(db=db):
    return (db

        # Small inward spiral.
        .spiral(
            turns=0.75,
            start_radius=0.70,
            end_radius=0.35,
            duration=4.2,
            direction="right")
        .corte(beat=0.20)
        # Long diagonal glide.
        .forward_right(distance=0.75, dt=2.5)
        # Immediate answer.
        .forward_left(distance=0.75, dt=2.5)
        # Flowing reverse figure-eight.
        .reverse_figure_eight(
            radius=0.65,
            duration=5.5)
        .corte(beat=0.30)
        # Open out ready for the reprise.
        .arc_left(
            radius=1.00,
            degrees=110,
            duration=3.0)
        .corte(beat=0.40)
    )

def reprise(db=db):
    return (db

        # Long confident promenade.
        .promenade(
            distance=1.20,
            duration=3.8)
        # Broad sweeping right-hand turn.
        .arc_right(
            radius=1.10,
            degrees=140,
            duration=4.2)
        # Flow straight into a figure eight.
        .figure_eight(
            radius=0.70,
            duration=5.5)
        # Brief hesitation on the musical accent.
        .corte(beat=0.25)
        # Continue travelling.
        .promenade(
            distance=0.90,
            duration=2.8)
        # Gentle diagonal drift.
        .forward_left(distance=0.40, dt=1.5)
        # Recover while keeping momentum.
        .backward_right(distance=0.20, dt=0.9)
        # Finish by opening towards the next phrase.
        .arc_left(
            radius=0.80,
            degrees=70,
            duration=2.4)
        .corte(beat=0.40)
    )

def crescendo(db=db):
    return (db

        # Large sweeping left-hand curve.
        .arc_left(
            radius=1.30,
            degrees=170,
            duration=5.0)
        # Continue directly into a long promenade.
        .promenade(
            distance=1.30,
            duration=3.6)
        # Open into a broad right-hand spiral.
        .spiral(
            turns=0.75,
            start_radius=1.20,
            end_radius=0.45,
            duration=4.5,
            direction="right")
        # A long diagonal glide.
        .forward_right(distance=0.80, dt=2.4)
        # Mirror it immediately.
        .forward_left(distance=0.80, dt=2.4)
        # Finish with a flowing quarter-circle.
        .arc_right(
            radius=0.90,
            degrees=90,
            duration=2.5)
        # Pause only when the phrase resolves.
        .corte(beat=0.45)
    )

def climax(db):
    return (db

        # Open with a broad outward spiral.
        .spiral(
            turns=1.0,
            start_radius=0.45,
            end_radius=1.30,
            duration=5.5,
            direction="left")
        # Continue immediately into a large figure eight.
        .figure_eight(
            radius=0.90,
            duration=6.0)
        # Long sweeping right-hand arc.
        .arc_right(
            radius=1.25,
            degrees=180,
            duration=4.8)
        # Strong promenade across the floor.
        .promenade(
            distance=1.40,
            duration=3.8)
        # Gentle diagonal flourish.
        .forward_right(distance=0.50, dt=1.4)
        .backward_left(distance=0.25, dt=0.8)
        # Musical punctuation before the finale.
        .corte(beat=0.50)
    )

def final_sweep(db):
    return (db

        # Sweep away from the climax.
        .arc_left(
            radius=1.40,
            degrees=120,
            duration=4.2)
        # Long travelling promenade.
        .promenade(
            distance=1.50,
            duration=4.0)
        # Elegant sideways drift while maintaining heading.
        .glide(distance=0.30, dt=1.0)
        # Continue diagonally.
        .forward_right(distance=0.70, dt=2.2)
        # Mirror the movement.
        .forward_left(distance=0.70, dt=2.2)
        # Large sweeping curve back toward centre.
        .arc_right(
            radius=1.20,
            degrees=140,
            duration=4.0)
        # Gentle closing promenade.
        .promenade(
            distance=0.90,
            duration=2.8)
        .corte(beat=0.40)
    )

def coda(db):
    return (db

        # A quiet, confident promenade.
        .promenade(
            distance=0.60,
            duration=2.5)
        # Gentle closing curve.
        .arc_left(
            radius=0.80,
            degrees=60,
            duration=2.2)
        # Brief hesitation.
        .corte(beat=0.30)
        # Small diagonal adjustment.
        .forward_right(distance=0.25, dt=1.0)
        # Return to centre.
        .backward_left(distance=0.15, dt=0.8)
        # Face the audience.
        .turn_left(
            degrees=20,
            dt=1.0)
        # Final tiny advance.
        .promenade(
            distance=0.20,
            duration=1.0)
        # The final pose.
        .corte(beat=3.0)
    )

# Start to build up the choreography
def el_corte():
    db = DanceBuilder()

    intro(db)
    opening_A(db)
    opening_A_prime(db)
    bridge(db)
    development_A(db)
    development_B(db)
    development_C(db)
    reprise(db)
    crescendo(db)
    climax(db)
    final_sweep(db)
    coda(db)
    
    return db.build()

keyframes = el_corte()

# Returns True if the bootsel button or the frog's hand button are pressed.
def button_pressed() -> bool:
    return bootsel_button() or not hand.value()

# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

robot = Robot()

do_music = True

try:
    sleep(2)  # Allow time for the DFPlayer to initialize
    led.value(1)  # Turn on LED to indicate we're starting the test
    if do_music:
        player1 = DFPlayerMini(1, 4, 5)
        print("DFPlayer Mini Test")
        result = player1.select_source('sdcard')
        print(f"Select Source Result: {result}")
        result = player1.query_num_files()
        print(f"Number of Files: {result}")
        result = player1.set_volume(25)
        print(f"Set Volume Result: {result}")

    print("Press the BOOTSEL button to start the demo sequence...")
    led.value(1)  # Turn on LED to indicate we're starting the test
    while not button_pressed():
        pass  # Wait for button press to start

    print("...starting demo sequence")
    if do_music:
        result = player1.play(1)
        print(f"Play Result: {result}")
#        time.sleep(5) # wait for spoken intro to finish

    traj = Trajectory(keyframes)
    runner = TrajectoryRunner(robot, traj)

    while True:
        if not runner.update():
            robot.stop()
            break

        robot.update()
        time.sleep_ms(1)
except KeyboardInterrupt:
    print("Interrupted by user")
except Exception as e:
    print("Error:", e)
finally:
    robot.stop()
    print("Motors released")
    if do_music:
        result = player1.stop()
        print(f"Stop Result: {result}")
