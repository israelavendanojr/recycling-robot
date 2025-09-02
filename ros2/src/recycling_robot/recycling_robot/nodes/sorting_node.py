#!/usr/bin/env python3
"""
ROS2 Sorting Node — stepper motor positioning for bin sorting
Topic in:  /pipeline/classification_done   (std_msgs/String JSON)
Topic out: /pipeline/sorting_done          (std_msgs/String JSON)
"""

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Import the stepper controller
from recycling_robot.utils.stepper_motor_controller import StepperMotorController


class SortingNode(Node):
    def __init__(self):
        super().__init__("sorting_node")

        # ROS2 Parameters (simplified)
        self.declare_parameter("stepper_speed_delay", 0.002)           # step delay (lower = faster)
        self.declare_parameter("hold_position_after_sort", True)       # keep motor energized
        self.declare_parameter("return_to_home_after_sort", False)     # return to 0° after sort

        self.stepper_speed_delay = float(self.get_parameter("stepper_speed_delay").value)
        self.hold_position = bool(self.get_parameter("hold_position_after_sort").value)
        self.return_to_home = bool(self.get_parameter("return_to_home_after_sort").value)

        # Classification to angle mapping (0-180° arc, matches physical layout)
        self.bin_angles = {
            "trash": 0,        # forward
            "metal": 60,       # first wedge  
            "plastic": 120,    # left wedge
            "cardboard": 180,  # far left
        }

        # Motor initialization
        self.motor = StepperMotorController()
        self.motor.enable()
        
        # Home to 0° (trash bin) on startup - critical for angle tracking
        self.get_logger().info("[SortingNode] Moving to home (trash bin, 0°)")
        self.motor.go_to_angle(0, speed_delay=self.stepper_speed_delay)
        self.get_logger().info("[SortingNode] Home position set")

        # I/O
        self.sub = self.create_subscription(
            String, "/pipeline/classification_done", self._on_classification, 10
        )
        self.pub_done = self.create_publisher(String, "/pipeline/sorting_done", 10)

        # State
        self.sorting_busy = False
        self.last_hash = None

        self.get_logger().info("[SortingNode] Ready. Stepper motor positioning for bin sorting.")
        self.get_logger().info(f"[SortingNode] Bin angles: {self.bin_angles}")
        self.get_logger().info(f"[SortingNode] Speed delay: {self.stepper_speed_delay}s, Hold position: {self.hold_position}")

    # ---------------- Callbacks ----------------

    def _on_classification(self, msg: String):
        """Handle classification_done: move to appropriate bin angle, publish sorting_done."""
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"[SortingNode] Invalid JSON: {e}")
            return

        # Basic dedupe
        cls = payload.get("class", "unknown")
        ts = payload.get("timestamp", time.time())
        h = f"{cls}_{int(ts)}"
        if h == self.last_hash:
            self.get_logger().debug("[SortingNode] Duplicate classification; skipping.")
            return
        self.last_hash = h

        if self.sorting_busy:
            self.get_logger().warn("[SortingNode] Busy; skipping new request.")
            return

        self.sorting_busy = True
        try:
            # Log classification received
            self.get_logger().info(f"[SortingNode] Classification received: {cls}")
            
            # Get target angle for this material (default to trash/0° for unknown)
            target_angle = self.bin_angles.get(cls, 0)
            current_angle = self.motor.get_current_angle()
            
            self.get_logger().info(f"[SortingNode] Moving from {current_angle:.1f}° to {target_angle}° ({cls} bin)")
            
            # Guarantee motor is enabled before movement
            self.motor.enable()
            
            # Move to target angle
            self.motor.go_to_angle(target_angle, speed_delay=self.stepper_speed_delay)
            
            # Get final angle for verification
            final_angle = self.motor.get_current_angle()
            self.get_logger().info(f"[SortingNode] Positioning complete at {final_angle:.1f}°")

            # Optionally return to home position
            if self.return_to_home:
                time.sleep(0.5)  # Brief pause at target
                self.get_logger().info("[SortingNode] Returning to home position")
                self.motor.go_to_angle(0, speed_delay=self.stepper_speed_delay)
                final_angle = 0

            # Keep motor enabled for quick response unless configured otherwise
            if not self.hold_position:
                self.motor.disable()

            # Publish completion message
            done = {
                "status": "complete",
                "material": cls,
                "action": f"moved_to_{int(target_angle)}_degrees",
                "angle": final_angle,
                "timestamp": time.time(),
            }
            out = String()
            out.data = json.dumps(done)
            self.pub_done.publish(out)

        except Exception as e:
            self.get_logger().error(f"[SortingNode] Sorting failed: {e}")
        finally:
            self.sorting_busy = False

    # ---------------- Shutdown ----------------

    def destroy_node(self):
        try:
            self.get_logger().info("[SortingNode] Shutting down...")
            
            # Return to home position before shutdown for safety
            if hasattr(self, "motor") and self.motor.gpio_initialized:
                self.get_logger().info("[SortingNode] Returning to home before shutdown...")
                try:
                    self.motor.go_to_angle(0, speed_delay=self.stepper_speed_delay)
                    time.sleep(0.5)  # Brief pause at home
                    self.motor.disable()
                    self.get_logger().info("[SortingNode] Home position reached and motor disabled")
                except Exception as e:
                    self.get_logger().error(f"[SortingNode] Failed to return home: {e}")
            
            # Clean up motor resources
            if hasattr(self, "motor"):
                self.motor.cleanup()
                
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SortingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception as e:
            print(f"Destroy error: {e}")
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
