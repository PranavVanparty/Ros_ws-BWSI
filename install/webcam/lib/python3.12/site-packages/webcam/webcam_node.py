import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class WebCamNode(Node):
    def __init__(self):
        super().__init__("web_cam_node")
        self.bridge = CvBridge()
        self.cam_pub = self.create_publisher(Image, "web_camera/image_raw", 10)

        self.get_logger().info("Connecting to webcam stream...")
        self.video = cv2.VideoCapture("udp://@:1234", cv2.CAP_FFMPEG)

        # Wait for a valid frame (with timeout)
        for _ in range(50):  # ~5 seconds
            ret, frame = self.video.read()
            if ret and frame is not None and frame.shape[0] > 0:
                self.get_logger().info("Webcam stream locked in.")
                break
            time.sleep(0.1)
        else:
            self.get_logger().error("Failed to lock onto webcam stream.")
            return

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        ret, frame = self.video.read()
        if ret and frame is not None and frame.shape[0] > 0:
            self.get_logger().info(f"Got frame: {frame.shape}")
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.cam_pub.publish(img_msg)
        else:
            self.get_logger().warn("No frame received or invalid shape.")

    def destroy_node(self):
        self.video.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = WebCamNode()
    if node.video.isOpened():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()