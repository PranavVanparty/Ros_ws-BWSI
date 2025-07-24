import numpy as np
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from linearReg import linearReg
from cv_bridge import CvBridge

class pathfinder(Node):
    def __init__(self):
        super().__init__("pathfinder")
        self.camera_sub = self.create_subscription(Image, "camera_feed/camera/image_raw", self.camera_callback, 10)
        self.processed_image_pub = self.create_publisher(Image, "processed_image", 10)
        self._logger.info("Pathfinder Node Initialized")
        self.linear_reg = linearReg()

    def camera_callback(self, img: Image):
        img = CvBridge.imgmsg_to_cv2(img, "bgr8")
        img = cv2.imread(img, cv2.IMREAD_GRAYSCALE)
        kernel = np.ones((2, 2), np.uint8)

        _, img_thresheld = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
        img_thresheld = cv2.erode(img_thresheld, kernel, iterations=1)
        img_thresheld = cv2.dilate(img_thresheld, kernel, iterations=1)
        img_thresheld = cv2.erode(img_thresheld, kernel, iterations=1)

        contours , hierarchy = cv2.findContours(img_thresheld, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        new_img = np.zeros(img.shape, dtype=np.uint8)

        if contours is not None and len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:
                new_img = cv2.drawContours(new_img, [largest_contour], -1, (255, 255, 255), thickness=cv2.FILLED)
                p1, p2 = self.linear_reg.get_line(new_img)
                new_img = cv2.line(new_img, p1, p2, (0, 0, 255), 1)
                self.logger.info("Line detected and drawn on the image.")

        self.processed_image_pub.publish(new_img)

def main():
    rclpy.init()
    node = pathfinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()