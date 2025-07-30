import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import String as StringMsg
from std_msgs.msg import Float32

import cv2
from cv_bridge import CvBridge
import cv2.aruco as aruco

class AprilTags(Node):
    def __init__(self):
        super().__init__("april_tags")
        #self.camera_sub = self.create_subscription(Image, "/camera/image_raw", self.camera_callback , 10)
        self.camera_sub = self.create_subscription(ImageMsg, "web_camera/image_raw", self.camera_callback , 10)
        self.at_image_pub = self.create_publisher(ImageMsg, "/april_tags", 10)
        #self.pos_pub = self.create_publisher(StringMsg, "/drone_position", 10)

        self.pos_pub_x = self.create_publisher(Float32, "/drone_position/x", 10)
        self.pos_pub_y = self.create_publisher(Float32, "/drone_position/y", 10)
        self.pos_pub_z = self.create_publisher(Float32, "/drone_position/z", 10)

        self.bridge = CvBridge()
        
        # Create ArUco dictionary and detector parameters (compatible with older OpenCV)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
        self.aruco_params = aruco.DetectorParameters_create()
        
        self.get_logger().info("AprilTags Node Initialized")
        
        # Camera calibration parameters (convert to numpy arrays)
        self.camera_mtx = np.array([[344.38362872, 0, 300.05122152],
                                   [0, 342.21421392, 223.68023377],
                                   [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([-0.17758893, 0.38761326, -0.07189361, 0.02190879, -0.26175288], dtype=np.float32)
        
        

    def camera_callback(self, img: ImageMsg):
        try:
            # Convert ROS Image message to OpenCV numpy array
            cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
            
            x = Float32()
            y = Float32()
            z = Float32()
            
            rvecs, tvecs = self.get_tag(cv_img)
            
            if rvecs is not None and tvecs is not None:
                drone_translation = self.find_pos(rvecs, tvecs)
                # Flatten the array and convert to float for the message
                x.data = float(drone_translation.flatten()[0]) * (25/40)
                y.data = float(drone_translation.flatten()[1]) * (25/40)
                z.data = float(drone_translation.flatten()[2]) * (25/40)

            self.pos_pub_x.publish(x)
            self.pos_pub_y.publish(y)
            self.pos_pub_z.publish(z)

        except Exception as e:
            self.get_logger().error(f"Error in camera callback: {str(e)}")
            
            
    def get_tag(self, img):
        # Ensure we have a valid numpy array
        if not isinstance(img, np.ndarray):
            self.get_logger().error(f"Expected numpy array, got {type(img)}")
            return None, None
            
        # Convert BGR to grayscale for ArUco detection (if needed)
        if len(img.shape) == 3:
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray_img = img
            
        corners, ids, rejected = aruco.detectMarkers(gray_img, self.aruco_dict, parameters=self.aruco_params)
        if len(corners) == 0:
            self.get_logger().info("No AprilTags detected")
            return None, None
        if ids is None: 
            self.get_logger().info("No IDs found for detected AprilTags")
            return None, None
            
        self.get_logger().info(f"Detected {len(ids)} AprilTags with IDs: {ids.flatten()}")
            
        # Use the original color image for drawing (convert back to color if it was grayscale)
        img_color = img.copy() if len(img.shape) == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        img_color = aruco.drawDetectedMarkers(img_color, corners, ids)
        
        rvecs, tvecs = self.estimatePose(corners, self.camera_mtx, self.dist_coeffs)
        if rvecs is not None and tvecs is not None:
            img_color = cv2.drawFrameAxes(img_color, self.camera_mtx, self.dist_coeffs, rvecs, tvecs, 0.1)
        
        # Convert the annotated image back to ROS message
        self.at_image_pub.publish(self.bridge.cv2_to_imgmsg(img_color, "bgr8"))
        return rvecs, tvecs
        
        
    def estimatePose(self, corners, mtx, distortion, marker_size=26.6):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
            corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],#corners of AR in world frame
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        for c in corners:
            _, rvecs, tvecs = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        return rvecs, tvecs
    
    def find_pos(self, rvec, tvec):
        """rvec is a rodriguez vector and tvec is the position of the tag relative to
        the camera in camera frame. Use rvec to create a rotation matrix and find
        position of the drone relative to the tag; return the position of the drone. 
        Additionally, it may be helpful to have the orientation of the drone so you can could
        also return euler angles, the full rotation matrix, or some other information."""
        
        # Convert Rodriguez vector to rotation matrix
        rot_mat, _ = cv2.Rodrigues(rvec)
        
        # Calculate the drone position relative to the tag
        inv_rot_mat = -rot_mat
        drone_from_ar = np.dot(inv_rot_mat, tvec)
        
        return drone_from_ar
    
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