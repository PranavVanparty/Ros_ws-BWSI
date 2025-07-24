#!/usr/bin.env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
#if you import more libraries, add them to dependencies in package.xml
class PoseSubNode(Node):
  def __init__(self):
      super().__init__('pose_subscriber')
      self.get_logger().info('pose_subscriber has been started')
      self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
  def pose_callback(self, msg:Pose):
      self.get_logger().info('('+str(msg.x)+', '+str(msg.y))
      
def main(args=None):
  rclpy.init(args=args)
  node = PoseSubNode()
  rclpy.spin(node) #Loops the node
  rclpy.shutdown()




if __name__=='__main__':#Unneeded if you only ever run the node directly with ros2 run
  main()
