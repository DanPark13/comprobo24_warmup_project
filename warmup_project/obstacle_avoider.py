import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoiderNode(Node):
    """
    This is a node that avoids obstacles.
    """
    def __init__(self):
        super().__init__('obstacle_avoider_node')

        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.process_scan, 10)

        # Distance threshold for detecting an obstacle (in meters)
        self.obstacle_distance_threshold = 1.0

    def process_scan(self, msg):
        """
        Receives scan data, detects obstacles within the threshold distance,
        and steers away from obstacles.
        """
        # Extract ranges from the laser scan data
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Initialize variables to store the nearest obstacle's distance and angle
        closest_distance = float('inf')
        obstacle_angle = None

        # Detect the person
        person_angle = None
        for i, distance in enumerate(ranges):
            if distance < self.person_distance_max:
                person_angle = angle_min + i * angle_increment
                break

        # Command the robot to avoid obstacles
        self.avoid_obstacles(closest_distance, obstacle_angle)

    def avoid_obstacles(self, distance_to_obstacle, angle_to_obstacle):
        """
        Drives robot based on the detected obstacle's distance and angle.
        """
        # Create a velocity command
        msg = Twist()

        if distance_to_obstacle < self.obstacle_distance_threshold:
            # Obstacle detected and stop moving forward and turn away
            msg.linear.x = 0.0
            msg.angular.z = -angle_to_obstacle
            self.get_logger().info(f"Obstacle detected! Turning away from angle: {angle_to_obstacle}")
        else:
            # No obstacle nearby, move forward
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info("No obstacles. Moving forward...")

        # Publish the command
        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoider_node = ObstacleAvoiderNode()
    rclpy.spin(obstacle_avoider_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
