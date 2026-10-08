#!/usr/bin/env python3
"""
teleop.py

Keyboard teleoperation for the 4-wheel cart (2 x MDD10A, 4 x Pololu 37D).
Same pin assignments and direction logic as driver.py.

Keys (WASD or arrow keys):
    W / Up      forward   (hold)
    S / Down    reverse   (hold)
    A / Left    steer left   (hold together with W or S)
    D / Right   steer right  (hold together with W or S)
    K           brake (hold) - slows to a stop faster than letting go
    J           maximum PWM +2%
    L           maximum PWM -2%
    Ctrl+C      stop immediately and quit

How "holding" works over SSH
----------------------------
A terminal never sends key-release events. It only sends the character again
and again while a key auto-repeats. So a key counts as "held" for as long as
its characters keep arriving, and as "released" once they stop.

Two consequences:
  * Only the most recently pressed key auto-repeats. To turn, hold W (or S)
    first, then add A or D. The steering key keeps the throttle alive. When
    you let go of A/D, tap W/S again to carry on straight.
  * If no keys arrive (you let go, or the SSH link drops), the cart slows to
    a stop by itself.
"""

import os
import select
import signal
import sys
import termios
import time
import tty

import RPi.GPIO as GPIO

PWM_FREQUENCY = 1000  # Hz

# -----------------------------
# Speed settings
# -----------------------------
DEFAULT_MAX_PWM = 40.0   # used if you just press Enter at the prompt
PWM_CEILING = 100.0      # J can never raise maximum PWM above this
PWM_STEP = 2.0           # J / L increment
ADJUST_INTERVAL = 0.10   # s, minimum time between J / L steps while held

# Ramp rates, in PWM % per second
ACCEL_RATE = 60.0        # while W / S is held
COAST_RATE = 30.0        # after letting go of W / S
BRAKE_RATE = 100.0       # while K is held (must be faster than COAST_RATE)

# Tank turning: speed of the inside wheels relative to the outside wheels.
#   -1.0 = inside wheels counter-rotate (spin on the spot)
#    0.0 = inside wheels stop (pivot around one side)
#    0.5 = inside wheels at half speed (wide arc)
TURN_INNER_SCALE = -1.0

# -----------------------------
# Key timing
# -----------------------------
# After the first character of a key press, the keyboard waits (typically
# 0.25-0.5 s) before auto-repeat starts. FIRST_PRESS_TIMEOUT has to be longer
# than that delay, or motion stutters at the start of every press.
FIRST_PRESS_TIMEOUT = 0.60   # s
REPEAT_TIMEOUT = 0.20        # s, "released" this long after repeats stop
LOOP_DT = 0.02               # s, control loop period (50 Hz)

# -----------------------------
# Motor control pins (BCM)
# -----------------------------
# RIGHT driver
FR_PWM = 12
FR_DIR = 16
BR_PWM = 19
BR_DIR = 26

# LEFT driver
FL_PWM = 18
FL_DIR = 25
BL_PWM = 13
BL_DIR = 22

# -----------------------------
# Encoder pins (configured but not used here)
# -----------------------------
ENC_BR_A = 5
ENC_BR_B = 6
ENC_BL_A = 17
ENC_BL_B = 27
ENC_FR_A = 20
ENC_FR_B = 21
ENC_FL_A = 23
ENC_FL_B = 24

NEVER = float("-inf")


class Motor:
    def __init__(self, name: str, pwm_pin: int, dir_pin: int, side: str):
        self.name = name
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.side = side.upper()   # "R" or "L"
        self.pwm = None
        self.forward = None        # last direction written to the DIR pin
        self.duty = 0.0            # last duty written to the PWM pin

    def setup(self):
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)

        GPIO.output(self.dir_pin, GPIO.LOW)
        GPIO.output(self.pwm_pin, GPIO.LOW)

        self.pwm = GPIO.PWM(self.pwm_pin, PWM_FREQUENCY)
        self.pwm.start(0)

        print(f"✓ {self.name} initialized: PWM=GPIO{self.pwm_pin}, DIR=GPIO{self.dir_pin}, side={self.side}")

    def set_dir_raw(self, high: bool):
        GPIO.output(self.dir_pin, GPIO.HIGH if high else GPIO.LOW)

    def set_duty(self, duty: float):
        duty = max(0.0, min(100.0, float(duty)))
        self.pwm.ChangeDutyCycle(duty)
        self.duty = duty

    def set_signed(self, value: float):
        """
        value > 0 drives this wheel so the cart moves forward,
        value < 0 drives it in reverse. Magnitude is PWM duty in %.

        Direction logic (same as driver.py):
          Right side: HIGH=forward, LOW=reverse
          Left side mirrored accordingly
        """
        duty = round(min(100.0, abs(value)), 1)

        if duty > 0:
            forward = value > 0
            if forward != self.forward:
                # The ramp always passes through zero, so the direction pin
                # only ever flips while duty is close to 0.
                self.forward = forward
                self.set_dir_raw(forward if self.side == "R" else not forward)

        if duty != self.duty:
            self.set_duty(duty)

    def stop(self):
        if self.pwm is not None:
            self.set_duty(0)
        GPIO.output(self.dir_pin, GPIO.LOW)
        GPIO.output(self.pwm_pin, GPIO.LOW)

    def cleanup(self):
        try:
            self.stop()
            if self.pwm is not None:
                self.pwm.stop()
        except Exception:
            pass


class KeyHold:
    """Tracks whether a key is being held, based on when its characters arrive."""

    def __init__(self):
        self.last = NEVER
        self.expires = NEVER

    def press(self, now: float):
        repeating = (now - self.last) < FIRST_PRESS_TIMEOUT
        self.last = now
        self.expires = now + (REPEAT_TIMEOUT if repeating else FIRST_PRESS_TIMEOUT)

    def extend(self, until: float):
        self.expires = max(self.expires, until)

    def active(self, now: float) -> bool:
        return now < self.expires

    def clear(self):
        self.last = NEVER
        self.expires = NEVER


def setup_gpio():
    print("\nSetting up GPIO...")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Configure encoders safely
    encoder_pins = [
        ENC_BR_A, ENC_BR_B,
        ENC_BL_A, ENC_BL_B,
        ENC_FR_A, ENC_FR_B,
        ENC_FL_A, ENC_FL_B,
    ]
    for pin in encoder_pins:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    back_right  = Motor("BACK RIGHT",  BR_PWM, BR_DIR, side="R")
    back_left   = Motor("BACK LEFT",   BL_PWM, BL_DIR, side="L")
    front_right = Motor("FRONT RIGHT", FR_PWM, FR_DIR, side="R")
    front_left  = Motor("FRONT LEFT",  FL_PWM, FL_DIR, side="L")

    motors = [back_right, back_left, front_right, front_left]
    for m in motors:
        m.setup()

    return motors


def ask_initial_pwm() -> float:
    while True:
        raw = input(f"\nInitial maximum PWM, 0-{PWM_CEILING:.0f}% [{DEFAULT_MAX_PWM:.0f}]: ").strip()
        if raw == "":
            return DEFAULT_MAX_PWM
        try:
            value = float(raw.rstrip("%"))
        except ValueError:
            print("Please enter a number.")
            continue
        if 0.0 <= value <= PWM_CEILING:
            return value
        print(f"Please enter a value between 0 and {PWM_CEILING:.0f}.")


ARROW_KEYS = {"A": "w", "B": "s", "C": "d", "D": "a"}  # up, down, right, left


def parse_keys(buf: str):
    """
    Turn raw terminal input into a list of key names.
    Letters are lower-cased; arrow keys are mapped to w / s / a / d.
    Returns (keys, leftover) where leftover is an incomplete escape sequence.
    """
    keys = []
    i = 0
    while i < len(buf):
        c = buf[i]
        if c == "\x1b":
            if i + 1 >= len(buf):
                break                      # wait for the rest of the sequence
            if buf[i + 1] not in "[O":
                i += 1                     # lone Esc, ignore it
                continue
            if i + 2 >= len(buf):
                break
            key = ARROW_KEYS.get(buf[i + 2])
            if key:
                keys.append(key)
            i += 3
        else:
            keys.append(c.lower())
            i += 1
    return keys, buf[i:]


def slew(current: float, target: float, max_step: float) -> float:
    """Move current toward target by at most max_step."""
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


def describe(braking, driving, throttle, steering, steer_dir, moving) -> str:
    if braking:
        return "BRAKING" if moving else "STOPPED"
    if driving:
        label = "FORWARD" if throttle > 0 else "REVERSE"
        if steering:
            label += " + LEFT" if steer_dir < 0 else " + RIGHT"
        return label
    return "COASTING" if moving else "STOPPED"


def drive_loop(fd: int, motors, max_pwm: float):
    drive = KeyHold()     # W / S
    steer = KeyHold()     # A / D
    brake = KeyHold()     # K
    adjust = KeyHold()    # J / L

    throttle = 0          # +1 forward, -1 reverse
    steer_dir = 0         # -1 left, +1 right
    left_cmd = 0.0        # signed PWM % currently applied to the left side
    right_cmd = 0.0       # signed PWM % currently applied to the right side

    last_adjust = NEVER
    last_label = None
    pending = ""
    prev = time.monotonic()

    while True:
        ready, _, _ = select.select([fd], [], [], LOOP_DT)
        now = time.monotonic()

        # ---- read keys ----
        if ready:
            data = os.read(fd, 256)
            if not data:
                raise EOFError("keyboard input closed")
            keys, pending = parse_keys(pending + data.decode("latin-1"))

            for k in keys:
                if k in ("w", "s"):
                    new = 1 if k == "w" else -1
                    if new != throttle:
                        drive.clear()
                        throttle = new
                    drive.press(now)
                    brake.clear()

                elif k in ("a", "d"):
                    new = -1 if k == "a" else 1
                    if new != steer_dir:
                        steer.clear()
                        steer_dir = new
                    steer.press(now)
                    # Only the newest key auto-repeats, so a held steering
                    # key is what keeps the throttle alive during a turn.
                    if drive.active(now):
                        drive.extend(steer.expires)

                elif k == "k":
                    brake.press(now)
                    drive.clear()
                    steer.clear()

                elif k in ("j", "l"):
                    adjust.press(now)
                    # Changing speed while driving should not interrupt motion.
                    for hold in (drive, steer):
                        if hold.active(now):
                            hold.extend(adjust.expires)

                    if now - last_adjust >= ADJUST_INTERVAL:
                        last_adjust = now
                        step = PWM_STEP if k == "j" else -PWM_STEP
                        new_max = max(0.0, min(PWM_CEILING, max_pwm + step))
                        if new_max > max_pwm:
                            print(f"PWM increased. Maximum PWM: {new_max:.0f}%", flush=True)
                        elif new_max < max_pwm:
                            print(f"PWM decreased. Maximum PWM: {new_max:.0f}%", flush=True)
                        else:
                            print(f"PWM at limit. Maximum PWM: {new_max:.0f}%", flush=True)
                        max_pwm = new_max

        # ---- work out what each side should be doing ----
        braking = brake.active(now)
        driving = drive.active(now) and not braking
        steering = steer.active(now) and driving

        left_target = 0.0
        right_target = 0.0

        if braking:
            rate = BRAKE_RATE
        elif driving:
            rate = ACCEL_RATE
            base = throttle * max_pwm
            left_target = base
            right_target = base
            if steering:
                if steer_dir < 0:
                    left_target = base * TURN_INNER_SCALE
                else:
                    right_target = base * TURN_INNER_SCALE
        else:
            rate = COAST_RATE

        # ---- ramp toward the targets ----
        dt = min(now - prev, 0.1)
        prev = now
        left_cmd = slew(left_cmd, left_target, rate * dt)
        right_cmd = slew(right_cmd, right_target, rate * dt)

        for m in motors:
            m.set_signed(left_cmd if m.side == "L" else right_cmd)

        # ---- status ----
        moving = abs(left_cmd) > 0.05 or abs(right_cmd) > 0.05
        label = describe(braking, driving, throttle, steering, steer_dir, moving)
        if label != last_label:
            print(label, flush=True)
            last_label = label


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    motors = []
    fd = sys.stdin.fileno()
    old_term = None

    # Stop the cart if the SSH session is closed or the process is killed.
    signal.signal(signal.SIGHUP, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)

    try:
        if not sys.stdin.isatty():
            raise RuntimeError("teleop.py must be run from an interactive terminal.")

        max_pwm = ask_initial_pwm()
        motors = setup_gpio()

        print("\n4-WHEEL KEYBOARD CONTROL | Ctrl+C to stop\n")
        print("  W / Up     forward        J   maximum PWM +2%")
        print("  S / Down   reverse        L   maximum PWM -2%")
        print("  A / Left   steer left     K   brake")
        print("  D / Right  steer right")
        print(f"\nMaximum PWM: {max_pwm:.0f}%\n")

        old_term = termios.tcgetattr(fd)
        tty.setcbreak(fd)   # keys arrive one at a time, no Enter, no echo

        drive_loop(fd, motors, max_pwm)

    except KeyboardInterrupt:
        print("\n\nStopped by user.")

    except EOFError:
        print("\n\nKeyboard input closed.")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Motors first, terminal second.
        try:
            for m in motors:
                m.cleanup()
            GPIO.cleanup()
        except Exception:
            pass
        if old_term is not None:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
            except Exception:
                pass
        print("✓ Driver GPIO cleanup complete.")


if __name__ == "__main__":
    main()