#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from lab_1.msg import scan_range
from std_msgs.msg import Header

class lidar:
    def __init__(self):
        self.subscriber_topic = None
        self.publisher_topic_1 = None
        self.publisher_topic_2 = None
        self.publisher_topic_3 = None
        self.pub_1 = None
        self.pub_2 = None
        self.pub_3 = None

    def lidar_callback(self,data):
        msg_1 = Float64()
        msg_2 = Float64()
        msg_3 = scan_range()

        closest_point = min(data.ranges)
        farthest_point = max(data.ranges)

        msg_1.data = closest_point
        msg_2.data = farthest_point

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "laser"
        msg_3.header = header
        msg_3.closest_point = closest_point
        msg_3.farthest_point = farthest_point

        self.pub_1.publish(msg_1)
        self.pub_2.publish(msg_2)
        self.pub_3.publish(msg_3)

    def run(self):
        rospy.init_node("lidar",anonymous=True)
        self.subscriber_topic = rospy.get_param('lab1_sub_topic')
        self.publisher_topic_1 = rospy.get_param("lab1_pub_topic1")
        self.publisher_topic_2 = rospy.get_param("lab1_pub_topic2")
        self.publisher_topic_3 = rospy.get_param("lab1_pub_topic3")

        self.pub_1 = rospy.Publisher(self.publisher_topic_1,Float64,queue_size = 0)
        self.pub_2 = rospy.Publisher(self.publisher_topic_2,Float64,queue_size = 0)
        self.pub_3 = rospy.Publisher(self.publisher_topic_3,scan_range,queue_size = 0)
        sub = rospy.Subscriber(self.subscriber_topic,LaserScan,self.lidar_callback)
        rospy.spin()

if __name__ == "__main__":
    lidar = lidar()
    lidar.run()
    
