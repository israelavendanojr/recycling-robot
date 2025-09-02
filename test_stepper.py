#!/usr/bin/env python3
import time
import lgpio

class StepperController:
    def __init__(self, dir_pin=17, step_pin=27, en_pin=22,
                 steps_per_rev=200, microsteps=1, gear_ratio=1.0):
        """
        Corrected for 1.8°/step motor (200 steps/rev).
        If you later enable microstepping or gearing, adjust microsteps/gear_ratio.
        """
        self.DIR_PIN = dir_pin
        self.STEP_PIN = step_pin
        self.EN_PIN = en_pin

        # Effective resolution
        self.STEPS_PER_REV = steps_per_rev
        self.MICROSTEPS = microsteps
        self.GEAR_RATIO = gear_ratio
        self.EFFECTIVE_STEPS = int(steps_per_rev * microsteps * gear_ratio)
        self.DEGREES_PER_STEP = 360.0 / self.EFFECTIVE_STEPS

        # Track current position
        self.current_angle = 0.0

        # Initialize GPIO
        self.h = lgpio.gpiochip_open(0)
        for pin in (self.DIR_PIN, self.STEP_PIN, self.EN_PIN):
            lgpio.gpio_claim_output(self.h, pin)

        # Safe state
        lgpio.gpio_write(self.h, self.STEP_PIN, 0)
        lgpio.gpio_write(self.h, self.DIR_PIN, 0)
        lgpio.gpio_write(self.h, self.EN_PIN, 1)  # disabled
        time.sleep(0.05)

        # Clear glitches
        for _ in range(5):
            lgpio.gpio_write(self.h, self.STEP_PIN, 1)
            time.sleep(0.00001)
            lgpio.gpio_write(self.h, self.STEP_PIN, 0)
            time.sleep(0.00001)

        print(f"Stepper initialized with {self.EFFECTIVE_STEPS} steps/rev "
              f"({self.DEGREES_PER_STEP:.4f}° per step)")

    def __del__(self):
        try:
            lgpio.gpio_write(self.h, self.EN_PIN, 1)
            lgpio.gpiochip_close(self.h)
        except:
            pass

    # ---------------- LOW-LEVEL ----------------
    def _pulse(self, delay):
        lgpio.gpio_write(self.h, self.STEP_PIN, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.h, self.STEP_PIN, 0)
        if delay > 0.00001:
            time.sleep(delay - 0.00001)

    def _update_angle(self, steps_moved, direction):
        if direction == 1:
            self.current_angle += steps_moved * self.DEGREES_PER_STEP
        else:
            self.current_angle -= steps_moved * self.DEGREES_PER_STEP
        self.current_angle %= 360

    def step_motor(self, steps, direction, delay_s=0.001):
        lgpio.gpio_write(self.h, self.DIR_PIN, direction)
        time.sleep(0.001)
        for _ in range(steps):
            self._pulse(delay_s)
        self._update_angle(steps, direction)

    # ---------------- ANGLE API ----------------
    def go_to_angle(self, target_angle, speed_delay=0.002, hold=True):
        target_angle %= 360
        angle_diff = target_angle - self.current_angle

        if angle_diff > 180: angle_diff -= 360
        elif angle_diff < -180: angle_diff += 360

        steps_needed = abs(int(round(angle_diff / self.DEGREES_PER_STEP)))
        if steps_needed == 0:
            print(f"Already at {self.current_angle:.1f}°")
            return

        direction = 1 if angle_diff > 0 else 0
        print(f"Moving {steps_needed} steps "
              f"{'CW' if direction else 'CCW'} "
              f"from {self.current_angle:.1f}° to {target_angle:.1f}°")

        self.step_motor(steps_needed, direction, speed_delay)
        print(f"Now at {self.current_angle:.1f}°")

        if not hold:
            self.disable_motor()

    # ---------------- UTILITIES ----------------
    def home_position(self):
        self.current_angle = 0.0
        print("Position reset to 0°")

    def get_current_angle(self):
        return self.current_angle

    def disable_motor(self):
        lgpio.gpio_write(self.h, self.EN_PIN, 1)
        print("Motor disabled")

    def enable_motor(self):
        lgpio.gpio_write(self.h, self.STEP_PIN, 0)
        lgpio.gpio_write(self.h, self.DIR_PIN, 0)
        time.sleep(0.001)
        lgpio.gpio_write(self.h, self.EN_PIN, 0)
        time.sleep(0.05)
        print("Motor enabled")


# --- Demo ---
if __name__ == "__main__":
    print("=== NEMA17 Sorting Chute Controller Demo ===")
    stepper = StepperController(dir_pin=17, step_pin=27, en_pin=22,
                                steps_per_rev=200, microsteps=1)

    try:
        stepper.enable_motor()

        time.sleep(.5)

        print("Calibrating quarter turns...")
        # Quarter turns
        stepper.go_to_angle(90)
        time.sleep(1)
        stepper.go_to_angle(180)
        time.sleep(1)
        stepper.go_to_angle(270)
        time.sleep(1)
        stepper.go_to_angle(0)
        time.sleep(1)

        # Calibration: one full revolution
        print("Calibrating one full revolution...")
        stepper.step_motor(200, direction=1, delay_s=0.005)
        time.sleep(.5)
        stepper.step_motor(200, direction=0, delay_s=0.005)
        time.sleep(.5)
        stepper.step_motor(200, direction=1, delay_s=0.005)
        time.sleep(.5)
        stepper.step_motor(200, direction=0, delay_s=0.005)
        time.sleep(.5)

        print("=== Demo Complete ===")
    finally:
        stepper.disable_motor()
