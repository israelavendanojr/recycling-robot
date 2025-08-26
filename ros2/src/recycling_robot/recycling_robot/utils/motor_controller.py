#!/usr/bin/env python3
"""
Real-time motor controller with multiple GPIO backend support
Optimized for sub-millisecond response times in recycling robot
"""
import time
import os
import threading
import struct
from contextlib import contextmanager
from typing import Optional, Tuple

class MotorController:
    """High-performance motor controller with failsafe GPIO access"""
    
    def __init__(self):
        # Motor driver pins (L298N)
        self.IN1_PIN = 17
        self.IN2_PIN = 27
        self.ENA_PIN = 22
        
        # State management
        self.pwm_value = 0.0
        self.pwm_thread = None
        self.pwm_running = False
        self.gpio_lib = None
        self.gpio_handle = None
        
        # Performance tracking
        self.last_command_time = 0
        
        # Initialize GPIO with fallback hierarchy
        self._initialize_gpio()
        
        # Initialize motor to stopped state
        self.stop()
        print(f"[MotorController] Initialized with {self.gpio_lib} backend")
    
    def _initialize_gpio(self):
        """Initialize GPIO with multiple fallback options"""
        
        # Method 1: lgpio (fastest, most reliable in containers)
        if self._try_lgpio():
            return
            
        # Method 2: RPi.GPIO with memory access fix
        if self._try_rpi_gpio():
            return
            
        # Method 3: Direct sysfs GPIO (always works, decent performance)
        if self._try_sysfs_gpio():
            return
            
        # Method 4: gpiozero (higher level, good fallback)
        if self._try_gpiozero():
            return
            
        # Final fallback: mock GPIO
        self._create_mock_gpio()
    
    def _try_lgpio(self) -> bool:
        """Try modern lgpio library"""
        try:
            import lgpio
            
            # Open GPIO chip
            self.gpio_handle = lgpio.gpiochip_open(0)
            
            # Test GPIO access by claiming pins
            lgpio.gpio_claim_output(self.gpio_handle, self.IN1_PIN)
            lgpio.gpio_claim_output(self.gpio_handle, self.IN2_PIN)
            lgpio.gpio_claim_output(self.gpio_handle, self.ENA_PIN)
            
            self.gpio_lib = 'lgpio'
            print(f"[MotorController] Using lgpio - GPIO pins {self.IN1_PIN}, {self.IN2_PIN}, {self.ENA_PIN}")
            
            # Start high-performance PWM thread
            self._start_software_pwm()
            return True
            
        except (ImportError, Exception) as e:
            print(f"[MotorController] lgpio failed: {e}")
            return False
    
    def _try_rpi_gpio(self) -> bool:
        """Try RPi.GPIO with Docker compatibility fixes"""
        try:
            import RPi.GPIO as GPIO
            
            # Force GPIO memory mapping for Docker containers
            if os.path.exists('/dev/gpiomem'):
                os.environ['GPIOMEM'] = '1'
            
            # Try different GPIO modes for container compatibility
            try:
                GPIO.setmode(GPIO.BCM)
            except RuntimeError:
                # If SOC detection fails, try manual setup
                GPIO.setmode(GPIO.BOARD)
                GPIO.setmode(GPIO.BCM)
            
            # Setup pins
            GPIO.setup(self.IN1_PIN, GPIO.OUT)
            GPIO.setup(self.IN2_PIN, GPIO.OUT)
            GPIO.setup(self.ENA_PIN, GPIO.OUT)
            
            # Create PWM instance
            self.pwm = GPIO.PWM(self.ENA_PIN, 1000)  # 1kHz frequency
            self.pwm.start(0)
            
            self.gpio_lib = 'RPi.GPIO'
            print(f"[MotorController] Using RPi.GPIO - GPIO pins {self.IN1_PIN}, {self.IN2_PIN}, {self.ENA_PIN}")
            return True
            
        except (ImportError, Exception) as e:
            print(f"[MotorController] RPi.GPIO failed: {e}")
            return False
    
    def _try_sysfs_gpio(self) -> bool:
        """Try direct sysfs GPIO access (works in any container)"""
        try:
            # Export GPIO pins via sysfs
            for pin in [self.IN1_PIN, self.IN2_PIN, self.ENA_PIN]:
                if not os.path.exists(f'/sys/class/gpio/gpio{pin}'):
                    with open('/sys/class/gpio/export', 'w') as f:
                        f.write(str(pin))
                
                # Set as output
                with open(f'/sys/class/gpio/gpio{pin}/direction', 'w') as f:
                    f.write('out')
            
            self.gpio_lib = 'sysfs'
            print(f"[MotorController] Using sysfs GPIO - GPIO pins {self.IN1_PIN}, {self.IN2_PIN}, {self.ENA_PIN}")
            
            # Start software PWM for ENA pin
            self._start_software_pwm()
            return True
            
        except (PermissionError, OSError, Exception) as e:
            print(f"[MotorController] sysfs GPIO failed: {e}")
            return False
    
    def _try_gpiozero(self) -> bool:
        """Try gpiozero library (good high-level fallback)"""
        try:
            from gpiozero import OutputDevice, PWMOutputDevice
            
            self.in1_device = OutputDevice(self.IN1_PIN)
            self.in2_device = OutputDevice(self.IN2_PIN)
            self.ena_device = PWMOutputDevice(self.ENA_PIN, frequency=1000)
            
            self.gpio_lib = 'gpiozero'
            print(f"[MotorController] Using gpiozero - GPIO pins {self.IN1_PIN}, {self.IN2_PIN}, {self.ENA_PIN}")
            return True
            
        except (ImportError, Exception) as e:
            print(f"[MotorController] gpiozero failed: {e}")
            return False
    
    def _create_mock_gpio(self):
        """Create mock GPIO for testing"""
        self.gpio_lib = 'mock'
        self.gpio_handle = None
        print("[MotorController] Using mock GPIO - motor will not move")
        print("[MotorController] WARNING: Physical motor will NOT move!")
    
    def _start_software_pwm(self):
        """Start high-performance software PWM thread"""
        self.pwm_running = True
        self.pwm_thread = threading.Thread(target=self._pwm_loop, daemon=True)
        self.pwm_thread.start()
    
    def _pwm_loop(self):
        """Optimized software PWM loop for precise timing"""
        period = 1.0 / 1000.0  # 1kHz PWM frequency
        
        while self.pwm_running:
            if self.pwm_value > 0:
                on_time = period * self.pwm_value
                off_time = period * (1.0 - self.pwm_value)
                
                if on_time > 0:
                    self._set_pin_state(self.ENA_PIN, True)
                    time.sleep(on_time)
                
                if off_time > 0:
                    self._set_pin_state(self.ENA_PIN, False)
                    time.sleep(off_time)
            else:
                self._set_pin_state(self.ENA_PIN, False)
                time.sleep(period)
    
    def _set_pin_state(self, pin: int, state: bool):
        """Set GPIO pin state using active backend"""
        if self.gpio_lib == 'lgpio':
            import lgpio
            lgpio.gpio_write(self.gpio_handle, pin, 1 if state else 0)
            
        elif self.gpio_lib == 'sysfs':
            try:
                with open(f'/sys/class/gpio/gpio{pin}/value', 'w') as f:
                    f.write('1' if state else '0')
            except Exception:
                pass  # Ignore write errors for robustness
    
    def _set_direction_pins(self, in1_state: bool, in2_state: bool):
        """Set direction pins based on GPIO library"""
        if self.gpio_lib == 'lgpio':
            import lgpio
            lgpio.gpio_write(self.gpio_handle, self.IN1_PIN, 1 if in1_state else 0)
            lgpio.gpio_write(self.gpio_handle, self.IN2_PIN, 1 if in2_state else 0)
            
        elif self.gpio_lib == 'RPi.GPIO':
            import RPi.GPIO as GPIO
            GPIO.output(self.IN1_PIN, in1_state)
            GPIO.output(self.IN2_PIN, in2_state)
            
        elif self.gpio_lib == 'sysfs':
            try:
                with open(f'/sys/class/gpio/gpio{self.IN1_PIN}/value', 'w') as f:
                    f.write('1' if in1_state else '0')
                with open(f'/sys/class/gpio/gpio{self.IN2_PIN}/value', 'w') as f:
                    f.write('1' if in2_state else '0')
            except Exception:
                pass
                
        elif self.gpio_lib == 'gpiozero':
            if in1_state:
                self.in1_device.on()
            else:
                self.in1_device.off()
            if in2_state:
                self.in2_device.on()
            else:
                self.in2_device.off()
                
        else:  # mock
            print(f"[MockGPIO] IN1={'HIGH' if in1_state else 'LOW'}, IN2={'HIGH' if in2_state else 'LOW'}")
    
    def _set_pwm(self, value: float):
        """Set PWM value (0.0 to 1.0)"""
        value = max(0.0, min(1.0, value))
        
        if self.gpio_lib in ['lgpio', 'sysfs']:
            self.pwm_value = value
            
        elif self.gpio_lib == 'RPi.GPIO':
            self.pwm.ChangeDutyCycle(value * 100)
            
        elif self.gpio_lib == 'gpiozero':
            self.ena_device.value = value
            
        else:  # mock
            print(f"[MockGPIO] ENA PWM = {value:.2f}")
    
    def forward(self, speed: float = 0.8, duration: Optional[float] = None):
        """Move motor forward at specified speed"""
        start_time = time.time()
        
        print(f"[MotorController] Motor forward (speed={speed:.2f}, lib={self.gpio_lib})")
        self._set_direction_pins(True, False)  # IN1=HIGH, IN2=LOW
        self._set_pwm(speed)
        
        # Track response time for performance monitoring
        self.last_command_time = (time.time() - start_time) * 1000
        
        if duration:
            time.sleep(duration)
            self.stop()
    
    def backward(self, speed: float = 0.8, duration: Optional[float] = None):
        """Move motor backward at specified speed"""
        start_time = time.time()
        
        print(f"[MotorController] Motor backward (speed={speed:.2f}, lib={self.gpio_lib})")
        self._set_direction_pins(False, True)  # IN1=LOW, IN2=HIGH
        self._set_pwm(speed)
        
        # Track response time for performance monitoring
        self.last_command_time = (time.time() - start_time) * 1000
        
        if duration:
            time.sleep(duration)
            self.stop()
    
    def stop(self):
        """Stop motor immediately"""
        start_time = time.time()
        
        print(f"[MotorController] Motor stop (lib={self.gpio_lib})")
        self._set_direction_pins(False, False)  # Both LOW
        self._set_pwm(0)
        
        # Track response time
        self.last_command_time = (time.time() - start_time) * 1000
    
    def get_status(self) -> dict:
        """Get motor controller status for diagnostics"""
        return {
            'gpio_library': self.gpio_lib,
            'is_mock': self.gpio_lib == 'mock',
            'pins': {
                'IN1': self.IN1_PIN,
                'IN2': self.IN2_PIN,
                'ENA': self.ENA_PIN
            },
            'last_response_time_ms': self.last_command_time,
            'pwm_running': self.pwm_running if hasattr(self, 'pwm_running') else False
        }
    
    def cleanup(self):
        """Clean up GPIO resources"""
        try:
            self.stop()
            
            if self.gpio_lib == 'lgpio':
                # Stop PWM thread
                self.pwm_running = False
                if self.pwm_thread and self.pwm_thread.is_alive():
                    self.pwm_thread.join(timeout=1.0)
                
                # Release GPIO
                import lgpio
                lgpio.gpiochip_close(self.gpio_handle)
                
            elif self.gpio_lib == 'RPi.GPIO':
                import RPi.GPIO as GPIO
                self.pwm.stop()
                GPIO.cleanup()
                
            elif self.gpio_lib == 'sysfs':
                # Stop PWM thread
                self.pwm_running = False
                if self.pwm_thread and self.pwm_thread.is_alive():
                    self.pwm_thread.join(timeout=1.0)
                
                # Unexport GPIO pins
                for pin in [self.IN1_PIN, self.IN2_PIN, self.ENA_PIN]:
                    try:
                        if os.path.exists(f'/sys/class/gpio/gpio{pin}'):
                            with open('/sys/class/gpio/unexport', 'w') as f:
                                f.write(str(pin))
                    except Exception:
                        pass
                        
            elif self.gpio_lib == 'gpiozero':
                self.in1_device.close()
                self.in2_device.close()
                self.ena_device.close()
            
            print("[MotorController] GPIO cleanup completed")
            
        except Exception as e:
            print(f"[MotorController] Cleanup error: {e}")

# High-performance test function
def benchmark_motor_controller():
    """Benchmark motor controller for real-time performance"""
    print("[Benchmark] Testing motor controller performance...")
    
    motor = MotorController()
    
    # Test initial status
    status = motor.get_status()
    print(f"[Benchmark] GPIO Library: {status['gpio_library']}")
    print(f"[Benchmark] Mock Mode: {status['is_mock']}")
    
    if status['is_mock']:
        print("[Benchmark] WARNING: Running in mock mode - no physical movement!")
    
    try:
        # Benchmark response times
        response_times = []
        
        print("[Benchmark] Testing response times...")
        for i in range(10):
            start = time.time()
            motor.forward(0.8, 0.01)  # Very short pulse
            response_time = (time.time() - start) * 1000
            response_times.append(response_time)
            time.sleep(0.01)
        
        avg_response = sum(response_times) / len(response_times)
        max_response = max(response_times)
        
        print(f"[Benchmark] Average response time: {avg_response:.2f}ms")
        print(f"[Benchmark] Maximum response time: {max_response:.2f}ms")
        print(f"[Benchmark] Sub-millisecond: {'✓' if avg_response < 1.0 else '✗'}")
        
        # Sorting simulation test
        print("[Benchmark] Simulating sorting decisions...")
        for i in range(5):
            # Simulate classification -> motor response
            start = time.time()
            motor.forward(0.9, 0.1)
            print(f"[Benchmark] Sort #{i+1} response: {(time.time() - start)*1000:.2f}ms")
            time.sleep(0.1)
        
        motor.stop()
        
        # Final status
        final_status = motor.get_status()
        print(f"[Benchmark] Final status: {final_status}")
        
    finally:
        motor.cleanup()

# Test function for direct execution
if __name__ == "__main__":
    benchmark_motor_controller()