import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tty
import select
import sys
import termios

class TeleOpNode(Node):
    """This is a node that runs the robot using WASDX"""
    def __init__(self):
        super().__init__("teleop_node")

        # Create publisher for motors
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Gets the system input
        settings = termios.tcgetattr(sys.stdin)
        key = None
        self.processKey(key, settings)
    
    def processKey(self, key, settings):
        """Processes key and commands robot to move"""
        msg = Twist()
        linearSpeed = 0.5
        angularSpeed = 0.5

        while key != '\x03':
            key = self.getKey(settings)
            print(key)
            # w key to move forward
            if key == '\x77':
                msg.linear.x = linearSpeed
                msg.angular.z = 0.0

            # a key to turn left
            elif key == '\x61':
                msg.linear.x = 0.0
                msg.angular.z = angularSpeed

            # x to go backwards
            elif key == '\x78':
                msg.linear.x = -linearSpeed
                msg.angular.z = 0.0

            # d to turn right
            elif key == '\x64':
                msg.linear.x = 0.0
                msg.angular.z = -angularSpeed

            # s to stop
            elif key == '\x73':
                msg.linear.x = 0.0
                msg.angular.z = 0.0

            self.vel_pub.publish(msg)
        
    def getKey(self, settings):
        """Gets key from keyboard (code provided by https://comprobo24.github.io/assignments/warmup_project#robot-teleoperation-teleop)"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleOpNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()