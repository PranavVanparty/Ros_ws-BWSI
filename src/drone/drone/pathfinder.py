import numpy as np
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class pathfinder(Node):
    def __init__(self):
        super().__init__("pathfinder")
        self.camera_sub = self.create_subscription(Image, "/camera/image_raw", self.camera_callback, 10)
        self.processed_image_pub = self.create_publisher(Image, "processed_image", 10)
        self.bridge = CvBridge()
        self.get_logger().info("Pathfinder Node Initialized")

    def camera_callback(self, img: Image):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        # Convert to grayscale
        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        kernel = np.ones((2, 2), np.uint8)
        _, img_thresheld = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)
        img_thresheld = cv2.erode(img_thresheld, kernel, iterations=1)
        img_thresheld = cv2.dilate(img_thresheld, kernel, iterations=1)
        img_thresheld = cv2.erode(img_thresheld, kernel, iterations=1)

        contours , hierarchy = cv2.findContours(img_thresheld, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        new_img = np.zeros(img_gray.shape, dtype=np.uint8)

        if contours is not None and len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:
                new_img = cv2.drawContours(new_img, [largest_contour], -1, (255, 255, 255), thickness=cv2.FILLED)
                p1, p2 = self.get_line(new_img)
                new_img = cv2.line(new_img, p1, p2, (0, 255, 0), 5)
                self.get_logger().info("Line detected and drawn on the image.")

        # Convert back to ROS Image message and publish
        processed_msg = self.bridge.cv2_to_imgmsg(new_img, "mono8")
        self.processed_image_pub.publish(processed_msg)

    def calc_regression(self,points):
        x, y = points[:,1], points[:,0]
        x_mean = np.mean(x)
        y_mean = np.mean(y)
        xy_mean = np.mean(x * y)
        x_squared_mean = np.mean(x ** 2)
        m = ((x_mean*y_mean) - xy_mean) / (np.square(x_mean)- x_squared_mean)
        b = y_mean - (m*x_mean)
        return m, b

    def get_inliners(self, m, b, shape):
        height, width = shape
        x1 = 0
        y1 = m*x1 + b
        if y1 > height or y1 < 0:
            y1 = height 
            x1 = (y1 - b) / m
        x2 = width - 1
        y2 = m*x2 + b
        if y2 > height or y2 < 0:
            y2 = height
            x2 = (y2 - b) / m
        return x1, y1, x2, y2

    def get_line(self, img):
        m,b = self.calc_regression(np.argwhere(img))
        x1,y1,x2,y2 = self.get_inliners(m,b,img.shape)
        return (int(x1),int(y1)),(int(x2),int(y2))

def main():
    rclpy.init()
    node = pathfinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
