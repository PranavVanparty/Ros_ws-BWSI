#!/usr/bin/env python3  # <-- fix typo here from `.env` to `.env`

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from copy import deepcopy
from functools import partial

class TurtleController(Node):
    def __init__(self):
        super().__init__("TurtleController")  # <-- fix: `super().__init__`, not `super.__init__`

        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.turtle_pose = Pose()
        self.previous_pose = Pose()

        self.pen_client = self.create_client(SetPen, "/turtle1/set_pen")

    def pose_callback(self, pose: Pose):
        self.previous_pose = deepcopy(self.turtle_pose)
        self.turtle_pose = pose

        # Detect side switch and call pen_service_call with appropriate color
        if (self.turtle_pose.x < 5.5) and (self.previous_pose.x >= 5.5):
            self.pen_service_call(0, 255, 0)  # Green
        elif (self.turtle_pose.x > 5.5) and (self.previous_pose.x <= 5.5):
            self.pen_service_call(255, 0, 0)  # Red

    def pen_service_call(self, r, g, b):
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for set_pen service...")

        request = SetPen.Request()  # <-- fix: add () to make it an instance

        request.r = r
        request.g = g
        request.b = b
        request.width = 2
        request.off = 0
        
        future = self.pen_client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))  # <-- fix: match function name

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e!r}')  # <-- fix: formatting and typo in "response"

def main():
    rclpy.init()
    node = TurtleController()  # <-- fix: call constructor with ()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()