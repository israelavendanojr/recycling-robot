#!/usr/bin/env python3
import time
import lgpio

# BCM GPIO pins
DIR_PIN = 17    # Direction
STEP_PIN = 27   # Step pulse
EN_PIN = 22     # Enable (LOW = enabled)

# Stepper specs
STEPS_PER_REV = 200    # 1.8° per step
MICROSTEPS = 1         # Adjust if MS1–3 are set
RPM = 60               # Target speed

# Calculate step delay
steps_per_second = (RPM * STEPS_PER_REV * MICROSTEPS) / 60
delay = 1.0 / steps_per_second / 2  # half-period

# Open GPIO chip
h = lgpio.gpiochip_open(0)

# Claim pins as outputs
lgpio.gpio_claim_output(h, DIR_PIN, 0)
lgpio.gpio_claim_output(h, STEP_PIN, 0)
lgpio.gpio_claim_output(h, EN_PIN, 1)  # start disabled

# Enable driver (LOW = enabled)
lgpio.gpio_write(h, EN_PIN, 0)
print("[StepperTest] Driver enabled")

# Set direction (0 = one way, 1 = the other)
lgpio.gpio_write(h, DIR_PIN, 0)

print(f"[StepperTest] Rotating {STEPS_PER_REV} steps ({360}°) at {RPM} RPM...")
for i in range(STEPS_PER_REV * MICROSTEPS):
    lgpio.gpio_write(h, STEP_PIN, 1)
    time.sleep(delay)
    lgpio.gpio_write(h, STEP_PIN, 0)
    time.sleep(delay)

print("[StepperTest] Done. Disabling driver.")
lgpio.gpio_write(h, EN_PIN, 1)  # disable driver

# Cleanup
lgpio.gpiochip_close(h)
