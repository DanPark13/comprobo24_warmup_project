import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class FiniteStateController(Node):
    """
    This is a node that avoids obstacles.
    """
    def __init__(self):
        super().__init__('finite_state_controller_node')

        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.process_scan, 10)

        # Distance threshold for detecting an obstacle (in meters)
        self.obstacle_distance_threshold = 1.0

        # Set up timers for turning
        self.start_timestamp = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(0.1, self.run_loop)

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

        # Check for obstacles within the threshold distance
        for i, distance in enumerate(ranges):
            if 0 < distance < self.obstacle_distance_threshold:
                if distance < closest_distance:
                    closest_distance = distance
                    obstacle_angle = angle_min + i * angle_increment

        # Command the robot to avoid obstacles
        self.avoid_obstacles(closest_distance, obstacle_angle)

    def avoid_obstacles(self, distance_to_obstacle, angle_to_obstacle):
        """
        Drives robot based on the detected obstacle's distance and angle.
        """
        # Create a velocity command
        msg = Twist()

        if distance_to_obstacle < self.obstacle_distance_threshold:
            # Obstacle detected, take evasive action
            msg.linear.x = 0.0  # Stop moving forward
            msg.angular.z = -angle_to_obstacle  # Turn away from the obstacle
            self.get_logger().info(f"Obstacle detected! Turning away from angle: {angle_to_obstacle}")
        else:
            # No obstacle nearby, drive in a square
            current_time = self.get_clock().now().nanoseconds
            delta = (current_time - self.start_timestamp) * (10 ** -9)
            print(delta)

            # Forwards
            if delta < 3:
                msg.linear.x = 0.3
                msg.angular.z = 0.0
            
            # Turns Left
            elif delta < 8.5:
                msg.linear.x = 0.0
                msg.angular.z  = 0.3
            
            # Forwards again
            elif delta < 11.5: 
                msg.linear.x = 0.3
                msg.angular.z = 0.0

            # Turns Left
            elif delta < 17:
                msg.linear.x = 0.0
                msg.angular.z = 0.3
            
            # Forwards
            elif delta < 20:
                msg.linear.x = 0.3
                msg.angular.z = 0.0
            
            # Turns Left
            elif delta < 25.5:
                msg.linear.x = 0.0
                msg.angular.z = 0.3
            
            # Forwards
            elif delta < 28.5:
                msg.linear.x = 0.3
                msg.angular.z = 0.0
            
            # Turns Left
            elif delta < 34:
                msg.linear.x = 0.0
                msg.angular.z = 0.3

            self.vel_pub.publish(msg)

            # Publish the command
            self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoider_node = FiniteStateController()
    rclpy.spin(obstacle_avoider_node)
    obstacle_avoider_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
