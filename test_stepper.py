#!/usr/bin/env python3
import time, lgpio

DIR_PIN = 17      # BCM
STEP_PIN = 27     # BCM
EN_PIN = 22       # LOW = enabled

h = lgpio.gpiochip_open(0)
for p in (DIR_PIN, STEP_PIN, EN_PIN):
    lgpio.gpio_claim_output(h, p)

lgpio.gpio_write(h, EN_PIN, 0)     # enable driver
lgpio.gpio_write(h, DIR_PIN, 0)    # one direction

# accelerate a bit, then steady
def step(delay_s, steps):
    for _ in range(steps):
        lgpio.gpio_write(h, STEP_PIN, 1); time.sleep(delay_s)
        lgpio.gpio_write(h, STEP_PIN, 0); time.sleep(delay_s)

# small ramp
for d in [0.001, 0.0008, 0.0006, 0.0005]:
    step(d, 200)

# steady run
step(0.0005, 2000)

lgpio.gpio_write(h, EN_PIN, 1)     # disable
lgpio.gpiochip_close(h)
