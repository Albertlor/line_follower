import rclpy
import cv2
import numpy as np

from rclpy.node import Node


class Line_Detector_Node(Node):
    def __init__(self):
        super.__init__("line_detector")