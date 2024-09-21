import rclpy
import numpy as np
import math
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3 # Neato control Messages
from sensor_msgs.msg import LaserScan

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower_node")

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
    
    def process_scan(self, msg):
        msg = Twist()
        ranges = msg.ranges
        angle_diff = 5
        left_angles = [90 - 2*angle_diff, 90 - angle_diff, 90 + angle_diff, 90  + 2*angle_diff]
        left_ranges = [ranges[90 - 2*angle_diff], ranges[90 - angle_diff], ranges[90 + angle_diff], ranges[90 + 2*angle_diff]]
        print(left_ranges)

        slopes = []
        for i in range(len(left_ranges) - 1):
            slopes.append(((left_ranges[i+1] - left_ranges[i])))
        threshold_value = 0.5

        wall_distance = 0.75
        correction_threshold = 0.2
        wall_present = True
        for slope in slopes:
            print(slope)
            if slope > threshold_value or math.isnan(slope) or slope == 0.0: # if not detecting a wall
                wall_present = False
                msg.angular.z = -0.3
                msg.linear.x = 0.0 
                break
        if wall_present: # if detecting a wall on the left
            if left_ranges[0] - wall_distance > correction_threshold: # if turned too far to the right, turn left
                msg.angular.z = 0.2
                msg.linear.x = 0.1
            elif left_ranges[0] - wall_distance < -correction_threshold: # if turned too far to the left, turn right
                msg.angular.z = -0.15
                msg.linear.x = 0.1
            else:
                msg.angular.z = 0.0
                msg.linear.x = 0.2
        print(wall_present)              

        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()