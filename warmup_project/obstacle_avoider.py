import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

class ObstacleAvoiderNode(Node):
    """
    This is a node that follows a person
    """
    def __init__(self):
        super().__init__('obstacle_follower_node')

        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.process_scan, 10)

        # Maximum distance for detecting a person
        self.person_distance_max = 1.5

    def process_scan(self, msg):
        """
        Receives scan data and if object in range than calculate distance and angle and follows person
        """
        # Extract ranges from the laser scan data
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Detect the person
        person_angle = None
        for i, distance in enumerate(ranges):
            if distance < self.person_distance_max:
                person_angle = angle_min + i * angle_increment
                break

        # Command the robot to follow the detected person
        if person_angle is not None:
            self.follow_person(person_angle)

    def follow_person(self, angle_to_person):
        """
        Drives robot with given instructions from process_scan
        """
        # Create a velocity command
        msg = Twist()
        
        # Move forward with a constant speed
        msg.linear.x = 0.5  # Adjust this value for desired speed
        
        # Turn towards the person
        msg.angular.z = -angle_to_person  # Negative to turn towards the person
        
        # Publish the command
        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    person_follower_node = ObstacleAvoiderNode()
    rclpy.spin(person_follower_node)
    person_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
