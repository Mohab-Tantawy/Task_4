#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
from sensor_msgs.msg import FluidPressure

class pressure_node(Node):
    def __init__(self):
        super().__init__('pressure_node')
        self.publisher_ = self.create_publisher(FluidPressure, '/pressure', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('Pressure node has been started.')
        

    def timer_callback(self):
        msg = FluidPressure()
        msg.fluid_pressure = float(random.randint(900, 1100)) # Simulated pressure value between 900 and 1100
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Pressure: "{msg.fluid_pressure:.1f}hPa"')
        
        
def main():
    rclpy.init()
    node = pressure_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
    