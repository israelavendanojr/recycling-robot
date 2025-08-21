#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
from picamera2 import Picamera2

class PicamPub(Node):
    def __init__(self):
        super().__init__("picam_publisher")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 10.0)
        self.declare_parameter("frame_id", "camera_link")
        w = int(self.get_parameter("width").value)
        h = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "/camera/image_raw", 10)

        self.picam2 = Picamera2()
        cfg = self.picam2.create_preview_configuration(
            main={"size": (w, h), "format": "RGB888"}
        )
        self.picam2.configure(cfg); self.picam2.start()
        self.timer = self.create_timer(1.0/max(1.0, fps), self.tick)
        self.get_logger().info(f"Picamera2 started {w}x{h}@{fps}")

    def tick(self):
        rgb = self.picam2.capture_array()
        if rgb is None or rgb.size == 0:
            return
        msg = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.picam2.stop(); self.picam2.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init(); n = PicamPub()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
