import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time
import sys
import select
import termios
import tty

class turtle_node(Node):
    def __init__(self):
        super().__init__('turtle_node')
        self.turtle1_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.turtle2_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        self.get_logger().info("Turtle node has been started.")
        self.get_logger().info("COMMANDS:")
        self.get_logger().info("Turtle 1: UP: forward, DOWN: backward, LEFT: left, RIGHT: right")
        self.get_logger().info("Turtle 2: W: forward, S: backward, A: left, D: right")
        self.get_logger().info("Space: stop both turtles, q: quit")

        self.running = True
        self.input_thread = threading.Thread(target=self.read_command)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.old_settings = termios.tcgetattr(sys.stdin)
    def get_key(self):
        try:
            tty.setcbreak(sys.stdin.fileno())
            key = sys.stdin.read(1)
            return key
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        

    def read_command(self):
        while self.running:
            try:
                key = self.get_key()
                self.process_command(key)
            except:
                break
    
    def process_command(self, key):
        msg1 = Twist()
        msg2 = Twist()
        # Turtle 1 controls
        if key == '\x1b':
            next_char1 = sys.stdin.read(1)
            next_char2 = sys.stdin.read(1)
            if next_char1 == '[':
                if next_char2 == 'A':
                    msg1.linear.x = 2.0
                    self.get_logger().info("Turtle 1 moving forward")
                elif next_char2 == 'B':
                    msg1.linear.x = -2.0
                    self.get_logger().info("Turtle 1 moving backward")
                elif next_char2 == 'C':
                    msg1.angular.z = -2.0
                    self.get_logger().info("Turtle 1 turning right")
                elif next_char2 == 'D':
                    msg1.angular.z = 2.0
                    self.get_logger().info("Turtle 1 turning left")

        # Turtle 2 controls
        elif key == 'w':
            msg2.linear.x = 2.0
            self.get_logger().info("Turtle 2 moving forward")
        elif key == 's':
            msg2.linear.x = -2.0    
            self.get_logger().info("Turtle 2 moving backward")
        elif key == 'a':
            msg2.angular.z = 2.0
            self.get_logger().info("Turtle 2 turning left")
        elif key == 'd':
            msg2.angular.z = -2.0
            self.get_logger().info("Turtle 2 turning right")
        # Stop both turtles
        elif key == ' ':
            msg1.linear.x = 0.0
            msg1.angular.z = 0.0
            msg2.linear.x = 0.0
            msg2.angular.z = 0.0
            self.get_logger().info("Both turtles stopped")
        # Quit
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
