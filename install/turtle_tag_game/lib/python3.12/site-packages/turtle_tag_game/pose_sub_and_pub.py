#!/usr/bin.env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from copy import deepcopy

#if you import more libraries, add them to dependencies in package.xml
class PoseSubNode(Node):
  def __init__(self):
      super().__init__('pose_subscriber')
  
      self.player_pose = Pose()
      self.bot_pose = Pose()

      self.old_player_pose = Pose()
      self.old_bot_pose = Pose()

      self.player_pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.player_callback, 10)
      self.bot_pose_subscriber = self.create_subscription(Pose, "/turtle2/pose", self.bot_callback, 10)

      self.refresh_rate = 0.1
      self.timer = self.create_timer(self.refresh_rate, self.get_player_vel)

      self.respose_pub = self.create_publisher(Pose, "BotPose", 10)



  def player_callback(self, player_pose:Pose):
      self.old_player_pose = deepcopy(self.player_pose)
      self.player_pose = player_pose
      self.post_respose()
      
  def bot_callback(self, bot_pose:Pose):
     self.old_bot_pose = deepcopy(self.bot_pose)
     self.bot_pose = bot_pose
     self.post_respose()
  
  def post_respose(self):
     self.respose_pub.publish()
     
  def get_player_vel(self):
     # I know that pose had velocites but I wanted to calculate them
     vel_x = (self.player_pose.x - self.old_player_pose.x) / self.refresh_rate
     vel_y = (self.player_pose.y - self.old_player_pose.y) / self.refresh_rate
     ang_vel = (self.player_pose.theta - self.old_player_pose.theta) / self.refresh_rate

def main(args=None):
  rclpy.init(args=args)
  node = PoseSubNode()
  rclpy.spin(node) #Loops the node
  rclpy.shutdown()




if __name__=='__main__':#Unneeded if you only ever run the node directly with ros2 run
  main()
