import rclpy
import numpy as np
import math
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3 # Neato control Messages
from sensor_msgs.msg import LaserScan
from statistics import mean

class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoider_node")

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
    
    def process_scan(self, msg): # look for clusters
        # front angle is zero degrees, left is 90, right is 270 
        
        clusters = self.find_clusters(msg)
        
        cluster_headings = list(clusters.keys())
        cluster_distances = list(clusters.values())
        msg = Twist()
        if len(clusters) < 1:
            msg.angular.z = 0.0
            msg.linear.x = 0.5
            print("No obstacles detected...")
        else:
            obj_distance = min(cluster_distances) # distance of nearest cluster
            heading_index = cluster_distances.index(obj_distance)
            obj_heading = cluster_headings[heading_index] # heading of nearest cluster
            
            max_ang_vel = 1.0
            max_lin_vel = 0.5
            min_distance = 0.5
            if obj_distance < 1.5:
                msg.linear.x = obj_distance * (max_lin_vel/2.0)
                msg.angular.z = obj_heading * (-max_ang_vel/45)
            else:
                msg.linear.x = max_lin_vel
                msg.angular.z = 0.0

        print("lin_vel: " + str(msg.linear.x))
        print("ang_vel: " + str(msg.angular.z))
        self.vel_pub.publish(msg)

    def find_clusters(self, msg):
        # get the angles and ranges between -45 (left) and +45 (right)
        param = 60 # dgree range so -param to +param angle sweep
        ranges = msg.ranges
        angles = range(len(msg.ranges))
        
        left_angles = range(param+1)
        left_angles = left_angles[::-1]
        right_angles = angles[360-param:360]
        right_angles = right_angles[::-1]
        angles = np.concatenate((left_angles, right_angles))# front angle's ranges from -45 (left) to 45(right)

        left_ranges = ranges[0:param+1]
        left_ranges = left_ranges[::-1]
        print(left_ranges)
        # rclpy.shutdown()
        right_ranges = ranges[360-param:360]
        right_ranges = right_ranges[::-1]
        ranges = np.concatenate((left_ranges, right_ranges))# front angles from -45 to 45
        
        
        print(ranges)
        # find clusters between -45 and +45
        clusters = {} # will hold the angles at the centers of each cluster as the keys, and the 
        # average range of each cluster
        temp_angles = []
        temp_ranges = []
        cluster_sizes = []
        for i in range(2*param + 1):
            if ranges[i] > 0:
                temp_angles.append(angles[i])
                temp_ranges.append(ranges[i])
            else:
                if len(temp_angles) > 2:
                    avg_range = mean(temp_ranges)
                    center_angle = temp_angles[math.ceil(len(temp_angles)/2)]
                    clusters[center_angle] = avg_range
                    cluster_sizes.append(len(temp_angles))
                    temp_angles = []
                    temp_ranges = []
        
        #print(clusters)
        return clusters


def main(args=None):
    rclpy.init(args=args) # initializes ROS connection
    node = ObstacleAvoiderNode() # create node of type Wall Follower
    rclpy.spin(node) # keeps the node from 'dying', nodes usually stop after doing their job once
    rclpy.shutdown()



if __name__ == '__main__':
    main()