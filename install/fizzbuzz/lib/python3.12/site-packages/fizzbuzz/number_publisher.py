#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8 as integer

class NumberPublihser(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.publisher = self.create_publisher(integer, "numbers", 10)
        time_period = 0.5
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.i = 0
        
    def timer_callback(self):
        msg = integer()
        msg.data = self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    number_publisher = NumberPublihser()
    rclpy.spin(number_publisher)
    number_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__" :
    main()