import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DriveSquareNode(Node):
    """This is a node that runs the robot using WASDX"""
    def __init__(self):
        super().__init__("drive_square_node")

        # Create publisher for motors
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Set up timers for turning
        self.start_timestamp = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(0.1, self.run_loop)

    def run_loop(self):
        msg = Twist()

        current_time = self.get_clock().now().nanoseconds
        delta = (current_time - self.start_timestamp) * (10 ** -9)
        print(delta)

        # Forwards
        if delta < 5:
            msg.linear.x = 0.3
            msg.angular.z = 0.0
        
        # Turns Left
        elif delta < 10.5:
            msg.linear.x = 0.0
            msg.angular.z  = 0.3
        
        # Forwards again
        elif delta < 15.5: 
            msg.linear.x = 0.3
            msg.angular.z = 0.0

        # Turns Left
        elif delta < 21:
            msg.linear.x = 0.0
            msg.angular.z = 0.3
        
        # Forwards
        elif delta < 26:
            msg.linear.x = 0.3
            msg.angular.z = 0.0
        
        # Turns Left
        elif delta < 31.5:
            msg.linear.x = 0.0
            msg.angular.z = 0.3
        
        # Forwards
        elif delta < 36.5:
            msg.linear.x = 0.3
            msg.angular.z = 0.0
        
        # Turns Left
        elif delta < 42:
            msg.linear.x = 0.0
            msg.angular.z = 0.3

        self.vel_pub.publish(msg)

        
def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()