#!/usr/bin/env python3
"""
Test script for the refactored MotorController
This script tests both real pigpio connection and mock fallback
"""

import sys
import os
import time

# Add the package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'recycling_robot'))

from recycling_robot.utils.motor_controller import MotorController

def test_motor_controller():
    """Test the motor controller functionality"""
    print("[Test] Testing MotorController...")
    
    try:
        # Test with default settings
        print("\n[Test] 1. Testing MotorController initialization...")
        motor = MotorController()
        
        print(f"[Test] MotorController initialized successfully")
        
        # Test movement methods
        print("\n[Test] 2. Testing movement methods...")
        
        # Test forward movement
        print("[Test]    Testing forward movement...")
        motor.forward(0.8, 0.5)  # Move forward at 80% speed for 0.5 seconds
        time.sleep(0.1)
        
        # Test backward movement
        print("[Test]    Testing backward movement...")
        motor.backward(0.8, 0.5)  # Move backward at 80% speed for 0.5 seconds
        time.sleep(0.1)
        
        # Test stop
        print("[Test]    Testing stop...")
        motor.stop()
        
        # Test manual control (like sorting node will do)
        print("\n[Test] 3. Testing classification response simulation...")
        print("[Test]    Simulating classification received...")
        motor.forward(0.8)
        time.sleep(1)
        motor.stop()
        print("[Test]    Classification response completed")
            
        # Test cleanup
        print("\n[Test] 4. Testing cleanup...")
        motor.cleanup()
        
        print("\n[Test] All tests passed! MotorController is working correctly.")
        
    except Exception as e:
        print(f"\n[Test] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True

def test_basic_functionality():
    """Test basic functionality with current controller"""
    print("\n[Test] Testing basic motor functionality...")
    
    try:
        # Test basic motor operations
        motor = MotorController()
        
        print(f"[Test] Motor controller working")
        
        # Test basic functionality
        motor.forward(0.8, 0.1)  # 80% speed for 0.1 seconds
        motor.stop()
        motor.cleanup()
        
        print("[Test] Basic motor tests passed!")
        return True
        
    except Exception as e:
        print(f"[Test] Basic motor test failed: {e}")
        return False

if __name__ == "__main__":
    print("[Test] MotorController Test Suite")
    print("=" * 40)
    
    # Test 1: Normal operation
    success1 = test_motor_controller()
    
    # Test 2: Basic functionality
    success2 = test_basic_functionality()
    
    print("\n" + "=" * 40)
    if success1 and success2:
        print("[Test] All tests passed! MotorController is ready for use.")
        sys.exit(0)
    else:
        print("[Test] Some tests failed. Please check the implementation.")
        sys.exit(1)
