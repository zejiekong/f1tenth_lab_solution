#!/usr/bin/env python
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
        proc_ranges = ranges
        window_size = 4
        range_index = 0
        while range_index < len(ranges):
            window_sample = proc_ranges[range_index:range_index+window_size]
            window_mean = np.mean(window_sample)
            proc_ranges[range_index:range_index+window_size] = [window_mean] * window_size
            range_index += window_size

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        max_gap_len = 0
        max_gap_index = 0
        max_gap_list = []
        start_index = 0
        end_index = 0
        while max_gap_index < len(free_space_ranges):
            if free_space_ranges[max_gap_index] != 0:
                max_gap_list.append(free_space_ranges[max_gap_index])
            else:
                if len(max_gap_list) > 0:
                    if len(max_gap_list) > max_gap_len:
                        max_gap_len = len(max_gap_list)
                    max_gap_list = []
            max_gap_index += 1
        return None
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """

        return None

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        closest_point = proc_ranges.index(min(proc_ranges))

        #Eliminate all points inside 'bubble' (set them to zero) 
        bubble_radius = 2
        free_space_ranges = proc_ranges
        free_space_ranges[closest_point-bubble_radius:closest_point + bubble_radius+1] = [0] * bubble_radius * 2

        #Find max length gap 
        start_index,end_index = self.find_max_gap(free_space_ranges)

        #Find the best point in the gap 
        self.find_best_point()

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        sub_drive_msg = AckermannDrive()
        sub_drive_msg.steering_angle = angle
        sub_drive_msg.steering_angle_velocity = angular_velocity
        sub_drive_msg.speed = velocity
        drive_msg.drive = sub_drive_msg

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
