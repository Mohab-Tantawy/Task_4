#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class timer_node(Node):
    def __init__(self):
        super().__init__('timer_node')
        self.publisher_ = self.create_publisher(Int16, '/Timer', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 10
        self.get_logger().info('Timer node has been started.')

    def timer_callback(self):
        
        
        msg = Int16()
        msg.data = self.count
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count -= 1
        if self.count < 0:
            self.get_logger().info('Time is UP!')
            self.timer.cancel()
        
def main():
    rclpy.init()
    node = timer_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
    