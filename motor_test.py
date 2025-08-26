from gpiozero import PWMOutputDevice, OutputDevice
from time import sleep

# Assign pins (change if you used different GPIOs)
IN1 = OutputDevice(17)   # Pi GPIO17 → L298N IN1
IN2 = OutputDevice(27)   # Pi GPIO27 → L298N IN2
ENA = PWMOutputDevice(22) # Pi GPIO22 → L298N ENA (PWM)

def forward(speed=0.8):
    print("Motor forward")
    IN1.on()
    IN2.off()
    ENA.value = speed  # 0.0 to 1.0

def backward(speed=0.8):
    print("Motor backward")
    IN1.off()
    IN2.on()
    ENA.value = speed

def stop():
    print("Motor stop")
    IN1.off()
    IN2.off()
    ENA.value = 0

# Demo
forward(0.5)   # spin forward 70% speed
sleep(2)
backward(1)  # spin backward 50% speed
sleep(2)
forward(.5)   # spin forward 70% speed
sleep(.5)
backward(0.25)   # spin forward 70% speed
sleep(1)
stop()
