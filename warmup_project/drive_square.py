import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DriveSquareNode(Node):
    """This node continuously drives the robot in a square."""
    def __init__(self):
        super().__init__("drive_square_node")

        # Create publisher for motors
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Set up timers for running the loop
        self.start_timestamp = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(0.1, self.run_loop)

    def run_loop(self):
        msg = Twist()

        current_time = self.get_clock().now().nanoseconds
        delta = (current_time - self.start_timestamp) * (10 ** -9)  # Time in seconds
        cycle_time = delta % 8.5  # Cycle is 3s forward + 5.5s turn
        
        # Forwards for the first 3 seconds of each cycle
        if cycle_time < 3:
            msg.linear.x = 0.3
            msg.angular.z = 0.0
        
        # Turn left for the next 5.5 seconds
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.3

        # Publish the velocity command
        self.vel_pub.publish(msg)

        
def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
