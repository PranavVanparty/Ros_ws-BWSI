import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from translation_center.srv import AvoidObstacle

class FindCenter(Node):
    def __init__(self):
        super().__init__("find_center")
        self.translation_service = self.create_service(AvoidObstacle, "translation", self.translation_callback)
        self.get_logger().info("Find Center Node Initialized")

        #! Put in correct positions
        #* The center will be the origin
        self.obstacle_dictionary = {
            #---Circle---# x,y,z translation TO the center. Relative to the tag.
            1: (10.0, 0.0, 0.0), 
            2: (0.0, -10.0, 0.0),
            3: (0.0, 0.0, -10.0),
            4: (0.0, 10.0, 0.0),
            #----ARC----#
            5: (5.0, 0.0, 0.0),
            6: (0.0, -5.0, 0.0),
            7: (0.0, 0.0, -5.0)
        }
        self.prev_error_z = 0.0
        self.integral_z = 0.0
        self.kp = 0.003
        self.ki = 0.001
        self.kd = 0.0005
        self.dt = 0.1  # Fixed change in time


    def translation_callback(self, request, response):
        #extract the drone's data from the request
        center_dist_tuple = self.obstacle_dictionary.get(request.tag)
        if center_dist_tuple is None:
            self.get_logger().error(f"Tag ID {request.tag} not found in obstacle dictionary")
            response.velocity_z = 0
            return response
            
        center_dist_z = center_dist_tuple[2]
        drone_z = request.drone_trans_z #translation from the drone to the tag in z-axis

        # Calculate the error in the z-axis
        error_z = center_dist_z - drone_z

        # --- PID Controller ---
        # Proportional term
        p_vel = self.kp * error_z

        # Integral term
        self.integral_z += error_z * self.dt
        i_vel = self.ki * self.integral_z

        # Derivative term
        derivative_z = (error_z - self.prev_error_z) / self.dt
        d_vel = self.kd * derivative_z

        # Update previous error
        self.prev_error_z = error_z

        # Combine all controllers
        velocity_z = p_vel + i_vel + d_vel

        # Set the response
        response.velocity_z = int(velocity_z)
        return response
    

def main(args=None):
    rclpy.init(args=args)
    find_center_node = FindCenter()
    rclpy.spin(find_center_node)
    find_center_node.destroy_node()
    rclpy.shutdown()   

if __name__ == "__main__":
    main()