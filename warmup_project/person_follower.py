import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower_node')
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Need to publish markers by seconds
        self.timer = self.create_timer(1.0, self.publish_marker)

    def publish_marker(self):
        """
        Create marker in rviz2
        """
        # Create a marker object
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "basic_shapes"
        marker.id = 0 
        marker.type = Marker.SPHERE
        
        # Set marker pose (position and orientation)
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the size of the marker (in meters)
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # Set the color (RGBA values between 0 and 1)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Lifetime of the marker (0 means it stays forever)
        marker.lifetime.sec = 0

        # Publish the marker
        self.marker_pub.publish(marker)
        self.get_logger().info('Published a marker at x=1.0, y=0.0')

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
