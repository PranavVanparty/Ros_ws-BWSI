import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import cv2.aruco as aruco

class AprilTags(Node):
    def __init__(self):
        super().__init__("april_tags")
        self.camera_sub = self.create_subscription(Image, "/camera_feed/camera/image_raw", self.camera_callback , 10)
        self.at_image_pub = self.create_publisher(Image, "april_tags", 10)
        self.bridge = CvBridge()
        self.aprilTag_detector = aruco.ArucoDetector()
        self.aprilTag_detector.setDictionary(self.aprilTag_detector.DICT_5x5_100)
        self.get_logger().info("AprilTags Node Initialized")

    def camera_callback(self, img: Image):
        img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        corners, ids, rejects = self.aprilTag_detector.detectMarkers(img, parameters=None)
        if ids is not None: 
            img = aruco.drawDetectedMarkers(img, corners, ids)
            

        
        