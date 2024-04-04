import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from example_interfaces.msg import String


class Line_Detector_Node(Node):
    def __init__(self):
        super().__init__("line_detector")
        self.publisher_ = self.create_publisher(String, "error_state", 10)
        self.timer_ = self.create_timer(0.5, self.publish_to_error_state)
        self.get_logger().info("Error is being published")

    def publish_to_error_state(self):
        msg = String()
        msg.data = "3.14"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Line_Detector_Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

        