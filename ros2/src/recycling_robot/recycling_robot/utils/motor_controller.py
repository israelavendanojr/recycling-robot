# recycling_robot/utils/motor_controller.py
# Simple motor controller using gpiozero
import time

class MotorController:
    def __init__(self):
        # Motor driver pins (L298N) - same as demo script
        try:
            from gpiozero import OutputDevice, PWMOutputDevice
            self.IN1 = OutputDevice(17)
            self.IN2 = OutputDevice(27) 
            self.ENA = PWMOutputDevice(22)
            print("[MotorController] Connected to GPIO pins 17, 27, 22")
        except Exception as e:
            print(f"[MotorController] GPIO not available: {e}")
            # Create mock objects for testing
            self.IN1 = MockOutputDevice("IN1")
            self.IN2 = MockOutputDevice("IN2")
            self.ENA = MockPWMOutputDevice("ENA")
            print("[MotorController] Using mock GPIO")
        
        # Initialize motor to stopped state
        self.stop()
    
    def forward(self, speed=0.8, duration=None):
        """Move motor forward at specified speed"""
        print(f"[MotorController] Motor forward (speed={speed})")
        self.IN1.on()
        self.IN2.off()
        self.ENA.value = speed
        
        if duration:
            time.sleep(duration)
            self.stop()
    
    def backward(self, speed=0.8, duration=None):
        """Move motor backward at specified speed"""
        print(f"[MotorController] Motor backward (speed={speed})")
        self.IN1.off()
        self.IN2.on()
        self.ENA.value = speed
        
        if duration:
            time.sleep(duration)
            self.stop()
    
    def stop(self):
        """Stop motor"""
        print("[MotorController] Motor stop")
        self.IN1.off()
        self.IN2.off()
        self.ENA.value = 0
    
    def cleanup(self):
        """Clean up - stop motor and release pins"""
        try:
            self.stop()
            if hasattr(self.IN1, 'close'):
                self.IN1.close()
            if hasattr(self.IN2, 'close'):
                self.IN2.close()
            if hasattr(self.ENA, 'close'):
                self.ENA.close()
            print("[MotorController] Cleanup completed")
        except Exception as e:
            print(f"[MotorController] Cleanup error: {e}")

class MockOutputDevice:
    """Mock GPIO output device for testing"""
    def __init__(self, name):
        self.name = name
        self.is_active = False
    
    def on(self):
        self.is_active = True
        print(f"[MockGPIO] {self.name} ON")
    
    def off(self):
        self.is_active = False
        print(f"[MockGPIO] {self.name} OFF")
    
    def close(self):
        print(f"[MockGPIO] {self.name} closed")

class MockPWMOutputDevice:
    """Mock PWM output device for testing"""
    def __init__(self, name):
        self.name = name
        self._value = 0
    
    @property
    def value(self):
        return self._value
    
    @value.setter
    def value(self, val):
        self._value = val
        print(f"[MockGPIO] {self.name} PWM = {val}")
    
    def close(self):
        print(f"[MockGPIO] {self.name} closed")