import rclpy 
from rclpy.node import Node

from fizzbuzz_interfaces.msg import FizzBuzz

from std_msgs.msg import Int8 as integer

class FizzBuzzNode(Node):
    def __init__(self):
        super().__init__('fizzbuzz')
        self.get_logger().info("starting fizzbuzz node")

        self.total_numbers = 0
        self.total_fizz = 0 
        self.total_buzz = 0
        self.total_fizzbuzz = 0

        self.fizzbuzz_pub = self.create_publisher(FizzBuzz, "fizzbuzz_stats", 10)

        self.number_sub = self.create_subscription(integer, "numbers", self.number_callback, 10)

    def number_callback(self, msg: integer) :
        number = msg.data
        fizzbuzz_str = self.fizzbuzz(number)
        fizzbuzz_msg = FizzBuzz()
        fizzbuzz_msg.fizzbuzz = fizzbuzz_str
        fizzbuzz_msg.fizz_ratio = self.total_fizz / self.total_numbers
        fizzbuzz_msg.buzz_ratio = self.total_buzz / self.total_numbers
        fizzbuzz_msg.fizzbuzz_ratio = self.total_fizzbuzz / self.total_numbers
        fizzbuzz_msg.number_total = self.total_numbers

        self.fizzbuzz_pub.publish(fizzbuzz_msg)

    def fizzbuzz(self, num) :
        if num % 3 == 0 :
            self.total_fizz += 1
            self.total_fizzbuzz +=1
            self.total_numbers += 1
            return "fizz"
        elif num % 5 == 0 :
            self.total_buzz += 1
            self.total_fizzbuzz += 1
            self.total_numbers += 1
            return "buzz"
        self.total_numbers += 1
        return ""

def main():
    rclpy.init()
    node = FizzBuzzNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

