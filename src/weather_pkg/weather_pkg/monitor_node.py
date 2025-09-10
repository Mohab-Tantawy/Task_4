#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, FluidPressure, RelativeHumidity
import datetime

class monitor_node(Node):
    def __init__(self):
        super().__init__('monitor_node')
        self.temp_sub = self.create_subscription(
            Temperature,
            '/temperature',
            self.temperature_callback,
            10)
        self.humi_sub = self.create_subscription(
            RelativeHumidity,
            '/humidity',
            self.humidity_callback,
            10)
        self.press_sub = self.create_subscription(
            FluidPressure,
            '/pressure',
            self.pressure_callback,
            10)
        self.current_temp = None
        self.current_humi = None    
        self.current_press = None

        self.data_file = "sensor_data.txt"
        self.init_data_file()

        self.get_logger().info('Monitor node has been started.')

    def init_data_file(self):
        """Initialize the data file with headers"""
        with open(self.data_file, 'w') as f:
            f.write("CurrentTime,Temperature (°C),Humidity (%),Pressure (hPa)\n")
            f.write("----------------------------------------------------\n")

    def save_to_file(self):
        """Save current readings to file with current time"""
        if None in [self.current_temp, self.current_humi, self.current_press]:
            return  # Don't save if any reading is missing
            
        currentTime = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        with open(self.data_file, 'a') as f:
            f.write(f"{currentTime},{self.current_temp:.2f},{self.current_humi:.2f},{self.current_press:.2f}\n")

    def temperature_callback(self, msg):
        self.current_temp = msg.temperature
        self.log_current_readings()
        self.save_to_file()

    def humidity_callback(self, msg):
        self.current_humi = msg.relative_humidity
        self.log_current_readings()
        self.save_to_file()

    def pressure_callback(self, msg):
        self.current_press = msg.fluid_pressure
        self.log_current_readings()
        self.save_to_file()

    def log_current_readings(self):
        temp_str = f"{self.current_temp:.2f} °C" if self.current_temp is not None else "N/A"
        humi_str = f"{self.current_humi:.2f} %" if self.current_humi is not None else "N/A"
        press_str = f"{self.current_press:.2f} hPa" if self.current_press is not None else "N/A"
        
        self.get_logger().info(
            f"Current Readings - Temp: {temp_str}, Humidity: {humi_str}, Pressure: {press_str}"
        )

def main():
    rclpy.init()
    node = monitor_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()