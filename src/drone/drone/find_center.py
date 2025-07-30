import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from translation_center.srv import translation_data

class FindCenter(Node):
    def __init__(self):
        super().__init__("find_center")
        self.translation_service = self.create_service(translation_data, "translation/y", self.translation_callback)
        self.get_logger().info("Find Center Node Initialized")
        self.april_tag_dictionary = {
            1: Float32(10),
            2: Float32(5),
            3: Float32(10)
        }
        self.prev_error = 0.0
        self.kp = 0.003
        self.dt = 0.1  # Fixed change in time


    def translation_callback(self, request, response):
        #extract the drone's data from the request
        center_dist = self.april_tag_dictionary(request.tag)
        drone_z = request.drone_trans_z #translation from the drone to the tag in z-axis

        # Calculate the error in the z-axis
        error_z = center_dist.data - drone_z

        # Calculate the velocity in the z-axis
        #Proportional controller
        
        velocity_z = error_z * self.kp 

        # integral controller
        integral_z = error_z * self.dt

        # Derivative controller
        derivative_z = (error_z - self.prev_error) / self.dt

        # Update previous error
        self.prev_error = error_z

        # Combine all controllers
        celocity_z = velocity_z + integral_z + derivative_z

        # Set the response
        response.velocity_z = velocity_z
        return response
    

def main(args=None):
    rclpy.init(args=args)
    find_center_node = FindCenter()
    rclpy.spin(find_center_node)
    find_center_node.destroy_node()
    rclpy.shutdown()   
     
if __name__ == "__main__":
    main()