import rclpy
import numpy as np
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower_node")

        # Create publisher for velocity commands
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        
        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)

    def process_scan(self, msg):
        """
        Main function that processes the laser scan data and controls the robot's movement.
        """
        ranges = msg.ranges
        
        # Get left range values from the laser scan
        left_ranges = self.get_left_ranges(ranges)

        # Calculate slopes between consecutive range values
        slopes = self.calculate_slopes(left_ranges)

        # Analyze the presence of a wall based on the slopes
        wall_present = self.is_wall_present(slopes)

        # Determine the robot's movement based on the wall distance and correction threshold
        twist_msg = self.control_robot_movement(wall_present, left_ranges)

        # Publish the calculated Twist message to the cmd_vel topic
        self.vel_pub.publish(twist_msg)

    def get_left_ranges(self, ranges):
        """
        Extracts the laser scan range values at the left of the robot.
        This function selects angles around 90 degrees (left side) and grabs their distance values.
        """
        angle_diff = 5  # Degree difference from 90 (left direction)
        left_angles = [90 - 2 * angle_diff, 90 - angle_diff, 90 + angle_diff, 90 + 2 * angle_diff]
        left_ranges = [ranges[90 - 2 * angle_diff], ranges[90 - angle_diff], ranges[90 + angle_diff], ranges[90 + 2 * angle_diff]]
        print(left_ranges)
        return left_ranges

    def calculate_slopes(self, left_ranges):
        """
        Calculates the difference (slope) between consecutive laser range values to estimate the wall's orientation.
        """
        slopes = []
        for i in range(len(left_ranges) - 1):
            slopes.append(left_ranges[i+1] - left_ranges[i])
        return slopes

    def is_wall_present(self, slopes):
        """
        Determines whether a wall is present on the robot's left side based on the slopes.
        """
        threshold_value = 0.5
        wall_present = True

        # Analyze slopes to detect if a wall is present
        for slope in slopes:
            if slope > threshold_value or math.isnan(slope) or slope == 0.0:
                wall_present = False
                break
        return wall_present

    def control_robot_movement(self, wall_present, left_ranges):
        """
        Determines the robot's movement (Twist message) based on the presence of a wall and the distance to it.
        """
        msg = Twist()
        wall_distance = 0.75  # Desired distance from the wall
        correction_threshold = 0.2  # Allowed deviation from the wall distance

        # If a wall is detected
        if wall_present:
            # Check if the robot needs to correct its path
            if left_ranges[0] - wall_distance > correction_threshold:  # Too far from the wall, turn left
                msg.angular.z = 0.2
                msg.linear.x = 0.1
            elif left_ranges[0] - wall_distance < -correction_threshold:  # Too close to the wall, turn right
                msg.angular.z = -0.15
                msg.linear.x = 0.1
            else:  # Correct distance, move forward
                msg.angular.z = 0.0
                msg.linear.x = 0.2
        else:
            # If no wall is detected, rotate to find one
            msg.angular.z = -0.3
            msg.linear.x = 0.0
        
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()