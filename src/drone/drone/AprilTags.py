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
        
        camera_mtx = [[344.38362872, 0, 300.05122152],
                      [0, 342.21421392, 223.68023377],
                      [0, 0, 1]]
        dist_coeffs = [[-0.17758893, 0.38761326, -0.07189361, 0.02190879, -0.26175288]]
        
        

    def camera_callback(self, img: Image):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            img_msg = cv2.cvtColor(img_msg, cv2.COLOR_BGR2GRAY)
            rvecs, tvecs = self.get_tag(img_msg)
            drone_translation = self.find_pos(rvecs, tvecs) if rvecs is not None and tvecs is not None else None
            
        except Exception as e:
            self.get_logger().error(f"Error in camera callback: {str(e)}")
            
            
    def get_tag(self, img: Image):
        corners, ids, rejected = aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)
        if len(corners) == 0:
            self.get_logger().info("No AprilTags detected")
            return None, None
        if ids is None: 
            self.get_logger().info("No IDs found for detected AprilTags")
            return None, None
            
        img = aruco.drawDetectedMarkers(img, corners, ids)
        
        rvecs , tvecs = self.estimatePose(corners, self.camera_mtx, self.dist_coeffs)
        img = aruco.drawAxis(img, self.camera_mtx, self.dist_coeffs, rvecs, tvecs, 0.1)
        
        self.at_image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
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
    
    def find_pos(rvec, tvec):
        """rvec is a rodriguez vector and tvec is the position of the tag relative to
        the camera in camera frame. Use rvec to create a rotation matrix and find
        position of the drone relative to the tag; return the position of the drone. 
        Additionally, it may be helpful to have the orientation of the drone so you can could
        also return euler angles, the full rotation matrix, or some other information."""
        
        a,b,y = rvec
        
        rot_mat = cv2.rodrigues(rvec)
        
        # Calculate the drone position relative to the tag
        inv_rot_mat = np.linalg.inv(rot_mat)
        drone_from_ar = -np.dot(inv_rot_mat, tvec)
        
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