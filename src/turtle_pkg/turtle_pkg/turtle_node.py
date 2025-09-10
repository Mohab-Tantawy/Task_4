import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time

class turtle_node(Node):
    def __init__(self):
        super().__init__('turtle_node')
        self.turtle1_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.turtle2_pub = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)

        self.get_logger().info("Turtle node has been started.")
        self.get_logger().info("COMMANDS:")
        self.get_logger().info("Turtle 1: UP: forward, DOWN: backward, LEFT: left, RIGHT: right")
        self.get_logger().info("Turtle 2: W: forward, S: backward, A: left, D: right")
        self.get_logger().info("Space: stop both turtles, q: quit")

        self.running = True
        self.input_thread = threading.Thread(target=self.read_command)
        self.input_thread.start()

    def read_command(self):
        while self.running:
            try:
                key = input().lower()
                self.process_command(key)
            except:
                break
    
    def process_command(self, key):
        msg1 = Twist()
        msg2 = Twist()

        if key == 'up':
            msg1.linear.x = 2.0
        elif key == 'down':
            msg1.linear.x = -2.0    
        elif key == 'left':
            msg1.angular.z = 2.0
        elif key == 'right':
            msg1.angular.z = -2.0

        elif key == 'w':
            msg2.linear.x = 2.0
        elif key == 's':
            msg2.linear.x = -2.0    
        elif key == 'a':
            msg2.angular.z = 2.0
        elif key == 'd':
            msg2.angular.z = -2.0
        elif key == ' ':
            msg1.linear.x = 0.0
            msg1.angular.z = 0.0
            msg2.linear.x = 0.0
            msg2.angular.z = 0.0
        elif key == 'q':
            self.running = False
            self.get_logger().info("Exiting...")
            rclpy.shutdown()
            return
        
        self.turtle1_pub.publish(msg1)
        self.turtle2_pub.publish(msg2)

def main():
    rclpy.init()
    node = turtle_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
