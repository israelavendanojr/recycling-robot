#!/usr/bin/env python3
"""
Stepper Motor Controller for Recycling Robot
Precise positioning for sorting bins using NEMA17 stepper motor
Backend: lgpio for reliable GPIO control in containers
"""

import time
import lgpio
import threading
from typing import Optional


class StepperMotorController:
    """High-precision stepper motor controller for bin positioning"""
    
    def __init__(self, dir_pin=17, step_pin=27, en_pin=22,
                 steps_per_rev=200, microsteps=1, gear_ratio=1.0):
        """
        Initialize stepper motor controller
        
        Args:
            dir_pin: Direction control pin (default: 17)
            step_pin: Step pulse pin (default: 27) 
            en_pin: Enable pin (default: 22, LOW=enabled)
            steps_per_rev: Motor steps per revolution (default: 200 for NEMA17)
            microsteps: Microstepping multiplier (default: 1)
            gear_ratio: Gear reduction ratio (default: 1.0)
        """
        # Pin configuration
        self.DIR_PIN = dir_pin
        self.STEP_PIN = step_pin
        self.EN_PIN = en_pin
        
        # Motor specifications
        self.STEPS_PER_REV = steps_per_rev
        self.MICROSTEPS = microsteps
        self.GEAR_RATIO = gear_ratio
        self.EFFECTIVE_STEPS = int(steps_per_rev * microsteps * gear_ratio)
        self.DEGREES_PER_STEP = 360.0 / self.EFFECTIVE_STEPS
        
        # Position tracking
        self.current_angle = 0.0
        self._position_lock = threading.Lock()
        
        # GPIO state
        self.gpio_handle = None
        self.gpio_initialized = False
        
        # Initialize GPIO
        self._initialize_gpio()
        
        print(f"[StepperMotorController] Initialized with {self.EFFECTIVE_STEPS} steps/rev "
              f"({self.DEGREES_PER_STEP:.4f}° per step)")
    
    def _initialize_gpio(self):
        """Initialize GPIO with error handling"""
        try:
            # Open GPIO chip
            self.gpio_handle = lgpio.gpiochip_open(0)
            
            # Claim GPIO pins
            lgpio.gpio_claim_output(self.gpio_handle, self.DIR_PIN)
            lgpio.gpio_claim_output(self.gpio_handle, self.STEP_PIN)
            lgpio.gpio_claim_output(self.gpio_handle, self.EN_PIN)
            
            # Set safe initial state
            lgpio.gpio_write(self.gpio_handle, self.STEP_PIN, 0)
            lgpio.gpio_write(self.gpio_handle, self.DIR_PIN, 0)
            lgpio.gpio_write(self.gpio_handle, self.EN_PIN, 1)  # Disabled
            time.sleep(0.05)
            
            # Clear any potential glitches
            for _ in range(5):
                lgpio.gpio_write(self.gpio_handle, self.STEP_PIN, 1)
                time.sleep(0.00001)
                lgpio.gpio_write(self.gpio_handle, self.STEP_PIN, 0)
                time.sleep(0.00001)
            
            self.gpio_initialized = True
            print("[StepperMotorController] GPIO initialized successfully")
            
        except Exception as e:
            print(f"[StepperMotorController] GPIO initialization failed: {e}")
            print("[StepperMotorController] WARNING: Motor will not move!")
            self.gpio_initialized = False
    
    def enable(self):
        """Enable motor (energize coils)"""
        if not self.gpio_initialized:
            print("[StepperMotorController] Cannot enable - GPIO not initialized")
            return
        
        try:
            # Set pins to safe state before enabling
            lgpio.gpio_write(self.gpio_handle, self.STEP_PIN, 0)
            lgpio.gpio_write(self.gpio_handle, self.DIR_PIN, 0)
            time.sleep(0.001)
            
            # Enable motor (EN_PIN LOW = enabled)
            lgpio.gpio_write(self.gpio_handle, self.EN_PIN, 0)
            time.sleep(0.05)
            
            print("[StepperMotorController] Motor enabled")
            
        except Exception as e:
            print(f"[StepperMotorController] Enable failed: {e}")
    
    def disable(self):
        """Disable motor (de-energize coils)"""
        if not self.gpio_initialized:
            return
        
        try:
            lgpio.gpio_write(self.gpio_handle, self.EN_PIN, 1)  # Disabled
            print("[StepperMotorController] Motor disabled")
            
        except Exception as e:
            print(f"[StepperMotorController] Disable failed: {e}")
    
    def _pulse_step(self, delay):
        """Generate single step pulse"""
        if not self.gpio_initialized:
            return
        
        try:
            lgpio.gpio_write(self.gpio_handle, self.STEP_PIN, 1)
            time.sleep(0.00001)  # Minimum pulse width
            lgpio.gpio_write(self.gpio_handle, self.STEP_PIN, 0)
            if delay > 0.00001:
                time.sleep(delay - 0.00001)
                
        except Exception as e:
            print(f"[StepperMotorController] Step pulse failed: {e}")
    
    def _update_angle(self, steps_moved, direction):
        """Update current angle tracking"""
        with self._position_lock:
            if direction == 1:  # Clockwise
                self.current_angle += steps_moved * self.DEGREES_PER_STEP
            else:  # Counter-clockwise
                self.current_angle -= steps_moved * self.DEGREES_PER_STEP
            
            # Normalize to 0-360 range
            self.current_angle %= 360
    
    def go_to_angle(self, target_angle, speed_delay=0.002):
        """
        Move to specific angle with precise positioning
        
        Args:
            target_angle: Target angle in degrees (0-360)
            speed_delay: Delay between steps in seconds (lower = faster)
        """
        if not self.gpio_initialized:
            print("[StepperMotorController] Cannot move - GPIO not initialized")
            return
        
        # Normalize target angle
        target_angle %= 360
        
        with self._position_lock:
            current = self.current_angle
        
        # Calculate shortest path
        angle_diff = target_angle - current
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
        
        # Calculate steps needed
        steps_needed = abs(int(round(angle_diff / self.DEGREES_PER_STEP)))
        
        if steps_needed == 0:
            print(f"[StepperMotorController] Already at {current:.1f}°")
            return
        
        # Determine direction (1 = clockwise, 0 = counter-clockwise)
        direction = 1 if angle_diff > 0 else 0
        direction_name = "CW" if direction else "CCW"
        
        print(f"[StepperMotorController] Moving {steps_needed} steps {direction_name} "
              f"from {current:.1f}° to {target_angle:.1f}°")
        
        try:
            # Set direction
            lgpio.gpio_write(self.gpio_handle, self.DIR_PIN, direction)
            time.sleep(0.001)  # Direction setup time
            
            # Execute steps
            for _ in range(steps_needed):
                self._pulse_step(speed_delay)
            
            # Update position tracking
            self._update_angle(steps_needed, direction)
            
            with self._position_lock:
                final_angle = self.current_angle
            
            print(f"[StepperMotorController] Movement complete - now at {final_angle:.1f}°")
            
        except Exception as e:
            print(f"[StepperMotorController] Movement failed: {e}")
    
    def get_current_angle(self):
        """Get current tracked angle"""
        with self._position_lock:
            return self.current_angle
    
    def home_position(self):
        """Reset position tracking to 0° (does not move motor)"""
        with self._position_lock:
            self.current_angle = 0.0
        print("[StepperMotorController] Position reset to 0°")
    
    def get_status(self):
        """Get controller status for diagnostics"""
        with self._position_lock:
            current_angle = self.current_angle
        
        return {
            'gpio_initialized': self.gpio_initialized,
            'current_angle': current_angle,
            'steps_per_rev': self.EFFECTIVE_STEPS,
            'degrees_per_step': self.DEGREES_PER_STEP,
            'pins': {
                'DIR': self.DIR_PIN,
                'STEP': self.STEP_PIN,
                'EN': self.EN_PIN
            }
        }
    
    def cleanup(self):
        """Safe shutdown - disable motor and release GPIO"""
        try:
            print("[StepperMotorController] Cleaning up...")
            
            # Disable motor first
            self.disable()
            
            # Release GPIO resources
            if self.gpio_initialized and self.gpio_handle is not None:
                lgpio.gpiochip_close(self.gpio_handle)
                self.gpio_initialized = False
                
            print("[StepperMotorController] Cleanup completed")
            
        except Exception as e:
            print(f"[StepperMotorController] Cleanup error: {e}")
    
    def __del__(self):
        """Destructor - ensure cleanup"""
        try:
            self.cleanup()
        except:
            pass


# Test function for direct execution
def test_stepper_controller():
    """Test stepper motor controller functionality"""
    print("=== Stepper Motor Controller Test ===")
    
    controller = StepperMotorController()
    status = controller.get_status()
    
    print(f"GPIO Initialized: {status['gpio_initialized']}")
    print(f"Current Angle: {status['current_angle']:.1f}°")
    print(f"Steps per Revolution: {status['steps_per_rev']}")
    
    if not status['gpio_initialized']:
        print("WARNING: GPIO not initialized - motor will not move")
        return
    
    try:
        # Enable motor
        controller.enable()
        time.sleep(0.5)
        
        # Test quarter turns (bin positions)
        positions = [90, 180, 270, 0]  # metal, plastic, trash, cardboard
        position_names = ["metal", "plastic", "trash", "cardboard"]
        
        for angle, name in zip(positions, position_names):
            print(f"\n--- Moving to {name} bin ({angle}°) ---")
            controller.go_to_angle(angle)
            time.sleep(1)
        
        print("\n=== Test Complete ===")
        
    except KeyboardInterrupt:
        print("\nTest interrupted")
    finally:
        controller.cleanup()


if __name__ == "__main__":
    test_stepper_controller()
