#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic,LaserScan,self.lidar_callback) #TODO
        self.drive_pub = rospy.Publisher(drive_topic,AckermannDriveStamped,queue_size=0) #TODO
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = list(ranges)
        window_size = 10
        range_index = 0
        while range_index < len(ranges):
            window_sample = proc_ranges[range_index:range_index+window_size]
            window_mean = np.mean(window_sample)
            proc_ranges[range_index:range_index+window_size] = [window_mean] * window_size
            range_index += window_size
        proc_ranges = [10 if i > 10 else i for i in proc_ranges]
        proc_ranges = [0 if i < 0.8 else i for i in proc_ranges]
        return proc_ranges

    def find_max_gap(self, free_space_ranges,threshold):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        max_gap_len = 0
        max_gap_index = 0
        max_gap_list = []
        end_index = 0
        while max_gap_index < len(free_space_ranges):
            if free_space_ranges[max_gap_index] > threshold:
                max_gap_list.append(free_space_ranges[max_gap_index])
            else:
                if len(max_gap_list) > 0:
                    if len(max_gap_list) > max_gap_len:
                        end_index = max_gap_index - 1
                        max_gap_len = len(max_gap_list)
                    max_gap_list = []
            max_gap_index += 1
        if end_index == 0:
            threshold -= 0.3
            return self.find_max_gap(free_space_ranges,threshold)
        return end_index - max_gap_len + 1,end_index
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        best_point = start_i + ranges[start_i:end_i+1].index(max(ranges[start_i:end_i+1]))
        return best_point

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = list(data.ranges)
        start_index = int(math.radians(90)/data.angle_increment)
        end_index = int((math.radians(270)/data.angle_increment))+1
        ranges = ranges[start_index:end_index]
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        closest_point = ranges.index(min(ranges))

        #Eliminate all points inside 'bubble' (set them to zero) 
        bubble_radius = 10
        free_space_ranges = proc_ranges
        for i in range(closest_point-bubble_radius,closest_point+bubble_radius):
            free_space_ranges[i] = 0
        #free_space_ranges[closest_point-bubble_radius:closest_point + bubble_radius+1] = [0] * bubble_radius * 2
        z
        #Find max length gap 
        start_index,end_index = self.find_max_gap(free_space_ranges,3)

        #Find the best point in the gap 
        best_point_index = self.find_best_point(start_index,end_index,ranges)
        print(ranges[best_point_index])
        #Publish Drive message
        angle = best_point_index * data.angle_increment
        angle -= math.pi/2
        print(angle)
        drive_msg = AckermannDriveStamped()
        sub_drive_msg = AckermannDrive()
        sub_drive_msg.steering_angle = angle
        sub_drive_msg.steering_angle_velocity = 8
        if 0 <= math.degrees(abs(angle)) <= 10:
            velocity = 0.4
        elif 10 < math.degrees(abs(angle)) <= 20:
            velocity = 0.3
        else:
            velocity = 0.2
        sub_drive_msg.speed = velocity
        drive_msg.drive = sub_drive_msg
        self.drive_pub.publish(drive_msg)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
