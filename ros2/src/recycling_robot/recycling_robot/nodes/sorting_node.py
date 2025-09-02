#!/usr/bin/env python3
"""
ROS2 Sorting Node — one full stepper revolution per classification
Topic in:  /pipeline/classification_done   (std_msgs/String JSON)
Topic out: /pipeline/sorting_done          (std_msgs/String JSON)
"""

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Import the stepper controller (path matches your repo layout)
from recycling_robot.utils.stepper_motor_controller import StepperMotorController


class SortingNode(Node):
    def __init__(self):
        super().__init__("sorting_node")

        # ROS2 Parameters
        self.declare_parameter("sorting_delay", 0.5)                    # pause before actuation
        self.declare_parameter("stepper_speed_delay", 0.002)           # step delay (lower = faster)
        self.declare_parameter("hold_position_after_sort", True)       # keep motor energized
        self.declare_parameter("return_to_home_after_sort", False)     # return to 0° after sort

        self.sorting_delay = float(self.get_parameter("sorting_delay").value)
        self.stepper_speed_delay = float(self.get_parameter("stepper_speed_delay").value)
        self.hold_position = bool(self.get_parameter("hold_position_after_sort").value)
        self.return_to_home = bool(self.get_parameter("return_to_home_after_sort").value)

        # Classification to angle mapping (4 bins, removed glass for safety)
        self.bin_angles = {
            "cardboard": 0,     # straight through (default)
            "metal": 90,        # first bin
            "plastic": 180,     # second bin  
            "trash": 270,       # third bin
            # Any unknown classification defaults to cardboard
        }

        # Motor
        self.motor = StepperMotorController()
        self.motor.enable()

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

        # Basic dedupe (optional)
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
            
            # Get target angle for this material (default to cardboard/0° for unknown)
            target_angle = self.bin_angles.get(cls, 0)
            
            # Log target angle
            current_angle = self.motor.get_current_angle()
            self.get_logger().info(f"[SortingNode] Target angle: {target_angle}°")
            self.get_logger().info(f"[SortingNode] Current → target position: {current_angle:.1f}° → {target_angle}°")
            
            # Small delay before movement
            time.sleep(self.sorting_delay)

            # Move to target angle
            self.motor.go_to_angle(target_angle, speed_delay=self.stepper_speed_delay)
            
            # Get final angle for verification
            final_angle = self.motor.get_current_angle()
            
            # Log completion
            self.get_logger().info(f"[SortingNode] Positioning complete at {final_angle:.1f}°")

            # Optionally return to home position
            if self.return_to_home:
                time.sleep(0.5)  # Brief pause at target
                self.get_logger().info("[SortingNode] Returning to home position")
                self.motor.go_to_angle(0, speed_delay=self.stepper_speed_delay)
                final_angle = 0

            # Optionally disable motor to save power
            if not self.hold_position:
                self.motor.disable()

            # Publish completion message with new format
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
