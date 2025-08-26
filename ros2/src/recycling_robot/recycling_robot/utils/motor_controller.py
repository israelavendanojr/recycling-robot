# recycling_robot/utils/motor_controller.py
# This motor controller calls the host GPIO service via HTTP
# The host service runs on the Pi and controls GPIO directly
import requests
import time

class MotorController:
    def __init__(self, in1_pin=17, in2_pin=27, ena_pin=22, host_url="http://localhost:5001"):
        """Initialize motor controller that calls host GPIO service"""
        self.IN1 = in1_pin
        self.IN2 = in2_pin
        self.ENA = ena_pin
        self.host_url = host_url
        self.current_position = 0
        
        # Test connection to host service
        try:
            response = requests.get(f"{self.host_url}/health", timeout=5)
            if response.status_code == 200:
                print(f"[MotorController] Connected to host GPIO service at {self.host_url}")
                print(f"[MotorController] GPIO pins: IN1={self.IN1}, IN2={self.IN2}, ENA={self.ENA}")
            else:
                raise Exception(f"Host service returned status {response.status_code}")
        except Exception as e:
            raise RuntimeError(f"Cannot connect to host GPIO service at {self.host_url}: {e}")

    def move_to_bin(self, target_bin: int):
        """Move motor to target bin via host GPIO service"""
        if target_bin == self.current_position:
            print(f"[MotorController] Already at bin {target_bin}")
            return
        
        try:
            # Call host service to move motor
            response = requests.post(
                f"{self.host_url}/motor/move",
                json={"bin": target_bin},
                timeout=30  # Longer timeout for motor movement
            )
            
            if response.status_code == 200:
                result = response.json()
                if result.get("success"):
                    self.current_position = target_bin
                    print(f"[MotorController] {result['message']}")
                else:
                    raise Exception(result.get("error", "Unknown error"))
            else:
                raise Exception(f"Host service returned status {response.status_code}")
                
        except Exception as e:
            print(f"[MotorController] Error moving to bin {target_bin}: {e}")
            raise e

    def forward(self):
        """Move motor forward via host service"""
        try:
            response = requests.post(f"{self.host_url}/motor/forward", timeout=5)
            if response.status_code == 200:
                result = response.json()
                if result.get("success"):
                    print("[MotorController] Motor moving forward")
                else:
                    raise Exception(result.get("error", "Unknown error"))
            else:
                raise Exception(f"Host service returned status {response.status_code}")
        except Exception as e:
            print(f"[MotorController] Error moving forward: {e}")
            raise e

    def backward(self):
        """Move motor backward via host service"""
        try:
            response = requests.post(f"{self.host_url}/motor/backward", timeout=5)
            if response.status_code == 200:
                result = response.json()
                if result.get("success"):
                    print("[MotorController] Motor moving backward")
                else:
                    raise Exception(result.get("error", "Unknown error"))
            else:
                raise Exception(f"Host service returned status {response.status_code}")
        except Exception as e:
            print(f"[MotorController] Error moving backward: {e}")
            raise e

    def stop(self):
        """Stop motor via host service"""
        try:
            response = requests.post(f"{self.host_url}/motor/stop", timeout=5)
            if response.status_code == 200:
                result = response.json()
                if result.get("success"):
                    print("[MotorController] Motor stopped")
                else:
                    raise Exception(result.get("error", "Unknown error"))
            else:
                raise Exception(f"Host service returned status {response.status_code}")
        except Exception as e:
            print(f"[MotorController] Error stopping motor: {e}")
            raise e

    def get_position(self):
        """Get current motor position from host service"""
        try:
            response = requests.get(f"{self.host_url}/motor/position", timeout=5)
            if response.status_code == 200:
                result = response.json()
                if result.get("success"):
                    self.current_position = result.get("position", 0)
                    return self.current_position
                else:
                    raise Exception(result.get("error", "Unknown error"))
            else:
                raise Exception(f"Host service returned status {response.status_code}")
        except Exception as e:
            print(f"[MotorController] Error getting position: {e}")
            return self.current_position

    def cleanup(self):
        """Clean up - stop motor if moving"""
        try:
            self.stop()
            print("[MotorController] Cleanup completed")
        except Exception as e:
            print(f"[MotorController] Cleanup error: {e}")
