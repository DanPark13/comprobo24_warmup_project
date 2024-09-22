import rclpy
import numpy as np
import math
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3 # Neato control Messages
from sensor_msgs.msg import LaserScan
from statistics import mean

class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__("person_follower_node")

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
    
    def process_scan(self, msg): # look for clusters
        # front angle is zero degrees, left is 90, right is 270   
        ranges = msg.ranges
        angles = range(len(msg.ranges))
        clusters = {} # will hold the angles at the centers of each cluster as the keys, and the 
        # average range of each cluster
        temp_angles = []
        temp_ranges = []
        for angle in angles:
            if ranges[angle] > 0:
                temp_angles.append(angle)
                temp_ranges.append(ranges[angle])
            else:
                if len(temp_angles) > 2:
                    avg_range = mean(temp_ranges)
                    center_angle = temp_angles[math.ceil(len(temp_angles)/2)]
                    clusters[center_angle] = avg_range
                    temp_angles = []
                    temp_ranges = []
        
        #print(clusters)
        cluster_headings = list(clusters.keys())
        cluster_distances = list(clusters.values())
        msg = Twist()
        if len(clusters) < 2:
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            print("No person detected...")
        else:
            #look for feet
            cluster_spacings = []
            #look for distances between clusters to represent space between feet
            for index in range(len(cluster_headings)-1): 
                cluster_spacings.append(cluster_headings[index+1] - cluster_headings[index])
            min_space = min(cluster_spacings)
            min_space_index = cluster_spacings.index(min_space)
            if min_space <= 5 and ((min(cluster_distances)) in {cluster_headings[min_space_index], cluster_headings[min_space_index+1]}):
                heading = int(mean([cluster_headings[min_space_index], cluster_headings[min_space_index+1]]))
                obj_distance = ranges[heading]
            # could maybe comment out this else statement so that if there is no cluster resembling feet
            #  the robot does nothing but maybe spin in place
            else:
                obj_distance = min(cluster_distances) # distance of nearest cluster
                heading_index = cluster_distances.index(obj_distance)
                heading = cluster_headings[heading_index] # heading of nearest cluster
            max_ang_vel = 1.0
            max_lin_vel = 0.5
            min_distance = 0.5
            if heading > 180:
                heading = heading - 360 # to make right side of Neato, negative angles from 0 to -179
            

            if -15 <= heading <= 15: # if person is reasonably directly in front of Neato
                msg.angular.z = 0.0
                msg.linear.x = max_lin_vel
            else:
                #msg.angular.z = heading * (max_ang_vel/180)
                if heading < 0:
                    msg.angular.z = -max_ang_vel
                else:
                    msg.angular.z = max_ang_vel
                #self.vel_pub.publish(msg)
            print("heading: " + str(heading))
            print("obj_distance: " + str(obj_distance))
            if obj_distance < min_distance:
                if (-15 <= heading <= 15):
                    msg.linear.x = -0.1
                    msg.angular.z = 0.0
                    # if -15 <= ranges.index(obj_distance) <= 15:
                    #     msg.linear.x = -0.1
                    #     msg.angular.z = 0.0
                    # if -90 >= ranges.index(obj_distance) >= 90:
            else:
                if obj_distance > 1.5:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.2
                else: # if object is between 0.5 and 2.5 m away
                    obj_distance = obj_distance - min_distance
                    msg.linear.x = obj_distance * (max_lin_vel/1.5)
                
        print("lin_vel: " + str(msg.linear.x))
        print("ang_vel: " + str(msg.angular.z))
        self.vel_pub.publish(msg)





def main(args=None):
    rclpy.init(args=args) # initializes ROS connection
    node = PersonFollowerNode() # create node of type Wall Follower
    rclpy.spin(node) # keeps the node from 'dying', nodes usually stop after doing their job once
    rclpy.shutdown()



if __name__ == '__main__':
    main()