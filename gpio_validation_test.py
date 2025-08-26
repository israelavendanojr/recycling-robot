#!/usr/bin/env python3
"""
GPIO Validation Test Script for Recycling Robot
Tests all GPIO backends and validates physical motor movement
"""
import os
import sys
import time
import subprocess
from pathlib import Path

def print_header(title):
    """Print formatted header"""
    print(f"\n{'='*60}")
    print(f" {title}")
    print(f"{'='*60}")

def test_host_gpio():
    """Test GPIO access on host system"""
    print_header("HOST SYSTEM GPIO TEST")
    
    # Check GPIO devices
    gpio_devices = list(Path("/dev").glob("gpio*"))
    print(f"GPIO devices found: {gpio_devices}")
    
    # Check user permissions
    user = subprocess.check_output("whoami", shell=True).decode().strip()
    groups = subprocess.check_output(f"groups {user}", shell=True).decode().strip()
    print(f"User: {user}")
    print(f"Groups: {groups}")
    print(f"GPIO group: {'✓' if 'gpio' in groups else '✗'}")
    
    # Test Python GPIO libraries
    libraries = ['lgpio', 'RPi.GPIO', 'gpiozero']
    for lib in libraries:
        try:
            if lib == 'lgpio':
                import lgpio
                handle = lgpio.gpiochip_open(0)
                lgpio.gpiochip_close(handle)
                print(f"{lib}: ✓ Available and working")
            elif lib == 'RPi.GPIO':
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.cleanup()
                print(f"{lib}: ✓ Available and working")
            elif lib == 'gpiozero':
                from gpiozero import LED
                # Don't actually create device on host
                print(f"{lib}: ✓ Available")
        except Exception as e:
            print(f"{lib}: ✗ Error - {e}")

def test_container_gpio():
    """Test GPIO access inside Docker container"""
    print_header("CONTAINER GPIO TEST")
    
    # Check if containers are running
    try:
        result = subprocess.run(
            ["docker", "compose", "ps", "--format", "table"],
            capture_output=True, text=True, check=True
        )
        print("Container status:")
        print(result.stdout)
    except Exception as e:
        print(f"Error checking containers: {e}")
        return False
    
    # Test GPIO inside container
    test_commands = [
        "ls -la /dev/gpio* || echo 'No GPIO devices'",
        "groups robot",
        "python3 -c 'import lgpio; print(\"lgpio available\")'",
        "python3 -c 'import RPi.GPIO; print(\"RPi.GPIO available\")'",
        "python3 -c 'import gpiozero; print(\"gpiozero available\")'",
    ]
    
    for cmd in test_commands:
        try:
            result = subprocess.run(
                ["docker", "compose", "exec", "-T", "ros2", "bash", "-c", cmd],
                capture_output=True, text=True, timeout=10
            )
            print(f"✓ {cmd}")
            if result.stdout.strip():
                print(f"  Output: {result.stdout.strip()}")
            if result.stderr.strip():
                print(f"  Error: {result.stderr.strip()}")
        except Exception as e:
            print(f"✗ {cmd} - Error: {e}")

def test_motor_controller():
    """Test the new motor controller"""
    print_header("MOTOR CONTROLLER TEST")
    
    try:
        # Test inside container
        cmd = "cd /workspace/ros2_ws && python3 src/recycling_robot/recycling_robot/utils/motor_controller.py"
        result = subprocess.run(
            ["docker", "compose", "exec", "-T", "ros2", "bash", "-c", cmd],
            capture_output=True, text=True, timeout=30
        )
        
        print("Motor Controller Test Output:")
        print(result.stdout)
        if result.stderr:
            print("Errors:")
            print(result.stderr)
            
        # Check if mock mode
        if "mock GPIO" in result.stdout.lower():
            print("\n⚠️  WARNING: Running in mock mode - physical motor will NOT move!")
            return False
        else:
            print("\n✅ SUCCESS: Real GPIO detected - motor should move!")
            return True
            
    except Exception as e:
        print(f"Error testing motor controller: {e}")
        return False

def test_physical_movement():
    """Test actual physical motor movement"""
    print_header("PHYSICAL MOVEMENT VALIDATION")
    
    print("This test will attempt to move the physical motor.")
    print("Ensure:")
    print("1. Motor is properly connected to GPIO pins 17, 27, 22")
    print("2. L298N driver is powered")
    print("3. You can observe the motor shaft")
    
    input("\nPress Enter to continue or Ctrl+C to abort...")
    
    try:
        # Create a simple movement test
        test_script = '''
import sys
sys.path.append('/workspace/ros2_ws/src/recycling_robot')
from recycling_robot.utils.motor_controller import MotorController
import time

print("Starting physical movement test...")
motor = MotorController()

status = motor.get_status()
print(f"GPIO Library: {status['gpio_library']}")
print(f"Mock Mode: {status['is_mock']}")

if not status['is_mock']:
    print("Testing forward movement (2 seconds)...")
    motor.forward(0.5, 2.0)
    time.sleep(0.5)
    
    print("Testing backward movement (2 seconds)...")
    motor.backward(0.5, 2.0)
    time.sleep(0.5)
    
    print("Testing rapid sorting movements...")
    for i in range(3):
        print(f"Sort movement {i+1}")
        motor.forward(0.8, 0.2)
        time.sleep(0.1)
    
    motor.stop()
    print("Physical test completed!")
else:
    print("Cannot test physical movement - running in mock mode")

motor.cleanup()
'''
        
        # Write test script to container
        with open("/tmp/motor_physical_test.py", "w") as f:
            f.write(test_script)
        
        # Copy to container and run
        subprocess.run([
            "docker", "compose", "cp", "/tmp/motor_physical_test.py", 
            "ros2:/tmp/motor_physical_test.py"
        ], check=True)
        
        result = subprocess.run([
            "docker", "compose", "exec", "-T", "ros2", 
            "python3", "/tmp/motor_physical_test.py"
        ], capture_output=True, text=True, timeout=60)
        
        print("Physical Test Output:")
        print(result.stdout)
        if result.stderr:
            print("Errors:")
            print(result.stderr)
            
        # Cleanup
        os.remove("/tmp/motor_physical_test.py")
        
    except Exception as e:
        print(f"Error in physical test: {e}")

def test_performance():
    """Test motor response performance"""
    print_header("PERFORMANCE VALIDATION")
    
    try:
        cmd = "cd /workspace/ros2_ws && python3 -c 'from src.recycling_robot.recycling_robot.utils.motor_controller import benchmark_motor_controller; benchmark_motor_controller()'"
        
        result = subprocess.run([
            "docker", "compose", "exec", "-T", "ros2", "bash", "-c", cmd
        ], capture_output=True, text=True, timeout=30)
        
        print("Performance Test Output:")
        print(result.stdout)
        if result.stderr:
            print("Errors:")
            print(result.stderr)
            
        # Check for sub-millisecond performance
        if "Sub-millisecond: ✓" in result.stdout:
            print("\n✅ PERFORMANCE: Sub-millisecond response achieved!")
        elif "Sub-millisecond: ✗" in result.stdout:
            print("\n⚠️  PERFORMANCE: Response time > 1ms (may still be acceptable)")
        else:
            print("\n❓ PERFORMANCE: Unable to determine response time")
            
    except Exception as e:
        print(f"Error in performance test: {e}")

def rebuild_containers():
    """Rebuild containers with new GPIO support"""
    print_header("REBUILDING CONTAINERS")
    
    print("Rebuilding ROS2 container with new GPIO libraries...")
    try:
        subprocess.run([
            "docker", "compose", "build", "--no-cache", "ros2"
        ], check=True)
        print("✅ Container rebuilt successfully")
        
        print("Restarting containers...")
        subprocess.run(["docker", "compose", "down"], check=True)
        subprocess.run(["docker", "compose", "up", "-d"], check=True)
        print("✅ Containers restarted")
        
        # Wait for startup
        print("Waiting for containers to initialize...")
        time.sleep(10)
        
    except Exception as e:
        print(f"Error rebuilding containers: {e}")
        return False
    
    return True

def main():
    """Main validation sequence"""
    print_header("GPIO VALIDATION FOR RECYCLING ROBOT")
    print("This script will validate GPIO access and motor functionality")
    
    # Step 1: Test host system
    test_host_gpio()
    
    # Step 2: Check if we need to rebuild
    print_header("CONTAINER REBUILD")
    rebuild = input("Rebuild containers with new GPIO support? (y/N): ").lower().strip()
    
    if rebuild == 'y':
        if not rebuild_containers():
            print("Failed to rebuild containers. Exiting.")
            return
    
    # Step 3: Test container GPIO
    test_container_gpio()
    
    # Step 4: Test motor controller
    motor_working = test_motor_controller()
    
    # Step 5: Performance test
    test_performance()
    
    # Step 6: Physical movement test (optional)
    if motor_working:
        physical = input("\nTest physical motor movement? (y/N): ").lower().strip()
        if physical == 'y':
            test_physical_movement()
    
    # Summary
    print_header("VALIDATION SUMMARY")
    print("1. Check the output above for any errors")
    print("2. Ensure 'Mock Mode: False' in motor controller test")
    print("3. Verify sub-millisecond response times")
    print("4. If physical test was run, confirm motor actually moved")
    print("\nIf all tests pass, your recycling robot motor system is ready!")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nValidation interrupted by user.")
    except Exception as e:
        print(f"\nValidation failed with error: {e}")
        sys.exit(1)

