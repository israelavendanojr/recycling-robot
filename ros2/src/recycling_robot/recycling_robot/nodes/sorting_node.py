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
from recycling_robot.utils.motor_controller import StepperMotorController as MotorController


class SortingNode(Node):
    def __init__(self):
        super().__init__("sorting_node")

        # Parameters
        self.declare_parameter("sorting_delay", 0.5)    # small pause before actuation
        self.declare_parameter("stepper_speed", 0.6)    # 0..1 fraction of MAX_SPEED_RPM

        self.sorting_delay = float(self.get_parameter("sorting_delay").value)
        self.stepper_speed = float(self.get_parameter("stepper_speed").value)

        # Motor
        self.motor = MotorController()
        self.motor.enable()

        # I/O
        self.sub = self.create_subscription(
            String, "/pipeline/classification_done", self._on_classification, 10
        )
        self.pub_done = self.create_publisher(String, "/pipeline/sorting_done", 10)

        # State
        self.sorting_busy = False
        self.last_hash = None

        self.get_logger().info("[SortingNode] Ready. One full spin per classification.")

    # ---------------- Callbacks ----------------

    def _on_classification(self, msg: String):
        """Handle classification_done: spin one revolution, publish sorting_done."""
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
            time.sleep(self.sorting_delay)

            # Map speed to RPM
            max_rpm = self.motor.MAX_SPEED_RPM
            rpm = max(1.0, min(self.stepper_speed * max_rpm, max_rpm))

            self.get_logger().info(f"[SortingNode] Spinning one revolution at {rpm:.1f} RPM for {cls}")
            self.motor.spin_one_revolution(rpm=rpm, clockwise=True)

            done = {
                "status": "complete",
                "material": cls,
                "action": "spin_1_rev",
                "rpm": rpm,
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
