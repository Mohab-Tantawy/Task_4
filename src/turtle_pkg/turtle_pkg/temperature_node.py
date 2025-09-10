#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import temperature

class temperature_node(Node):
    def __init__(self):
        super().__init__('temprature_node')
        self.publisher_ = self.create_publisher(temperature, '/temperature', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get.logger().info('Temperature node has been started.')

    def timer_callback(self):

        msg = temperature()
        msg.temperature = random.randint(15, 40)  # Simulated temperature value between 15 and 40
        msg.variance = 0.5  
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing temperature : {msg.temperature:.2f}°C (variance: {msg.variance})')
                
        
        
def main():
    rclpy.init()
    node = temperature_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    