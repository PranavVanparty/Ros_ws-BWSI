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
        self.camera_sub = self.create_subscription(Image, "/camera/image_raw", self.camera_callback , 10)
        self.at_image_pub = self.create_publisher(Image, "/april_tags", 10)
        self.bridge = CvBridge()
        
        # Create ArUco dictionary and detector parameters (compatible with older OpenCV)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
        self.aruco_params = aruco.DetectorParameters_create()
        
        self.get_logger().info("AprilTags Node Initialized")
        

    def camera_callback(self, img: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(img, "bgr8")
            corners, ids, rejected = aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)
            if ids is not None: 
                img = aruco.drawDetectedMarkers(img, corners, ids)
            img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.at_image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Error in camera callback: {str(e)}")
            
    
    
def main():
    rclpy.init()
    april_tags_node = AprilTags()
    try:
        rclpy.spin(april_tags_node)
    except KeyboardInterrupt:
        pass
    finally:
        april_tags_node.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()