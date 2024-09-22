import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class FiniteStateController(Node):
    """
    This is a node that drives in a square and avoids obstacles.
    """
    def __init__(self):
        super().__init__('finite_state_controller')

        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Subscribe to laser scan data for obstacle detection
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.process_scan, 10)

        # Distance threshold for detecting an obstacle (in meters)
        self.obstacle_distance_threshold = 1.0

        # Initialize timestamps for driving in a square
        self.start_timestamp = self.get_clock().now().nanoseconds
        self.square_driving = True  # Flag for square driving mode
        self.timer = self.create_timer(0.1, self.drive_square)  # Timer for updating control loop

        # Store whether an obstacle was detected
        self.obstacle_detected = False

    def process_scan(self, msg):
        """
        Receives scan data, detects obstacles within the threshold distance,
        and steers away from obstacles if necessary.
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

        # Set the flag for obstacle detection
        self.obstacle_detected = closest_distance < self.obstacle_distance_threshold
        if self.obstacle_detected:
            self.avoid_obstacles(closest_distance, obstacle_angle)

    def avoid_obstacles(self, distance_to_obstacle, angle_to_obstacle):
        """
        Drives the robot away from the obstacle.
        """
        # Create a velocity command
        msg = Twist()

        # Obstacle detected, take evasive action
        if distance_to_obstacle < self.obstacle_distance_threshold:
            msg.linear.x = 0.0  # Stop moving forward
            msg.angular.z = -angle_to_obstacle  # Turn away from the obstacle
            self.get_logger().info(f"Obstacle detected! Turning away from angle: {angle_to_obstacle}")
        else:
            self.get_logger().info(f"No obstacle detected yet")

        # Publish the command
        self.vel_pub.publish(msg)

    def drive_square(self):
        """
        Controls the driving logic, alternating between square driving and obstacle avoidance.
        """
        msg = Twist()

        if self.obstacle_detected:
            # If an obstacle is detected, obstacle avoidance is already handled by process_scan
            return
        else:
            # If no obstacle is detected, continue with square driving
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

        # Publish the command for square driving
        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
