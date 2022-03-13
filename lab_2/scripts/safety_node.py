#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header,Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import threading
import math

# TODO: import ROS msg types and libraries

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        self.ttc_threshold = 0 
        # TODO: create ROS subscribers and publishers.
        self.scan_subscriber = threading.Thread(target=self.sub_scan)
        self.odom_subscriber = threading.Thread(target=self.sub_odom)
        self.brake_publisher = None
        self.brake_bool_publisher = None

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        angle = scan_msg.angle_min
        brake = False
        for i in scan_msg.ranges:
            if i != math.inf and i != math.nan:
                v_i = self.speed * math.cos(angle)
                if v_i > 0:
                    ttc = i / v_i
                    if ttc < self.ttc_threshold:
                        brake = True
                        brake_msg = AckermannDriveStamped()
                        header = Header()
                        header.stamp = rospy.Time.now()
                        brake_msg.header = header
                        brake_msg.drive.speed = 0
                        self.brake_publisher.publish(brake_msg)
                        break
            angle += scan_msg.angle_increment

        # TODO: publish brake message and publish controller bool
        brake_bool_msg = Bool()
        brake_bool_msg.data = brake
        self.brake_bool_publisher.publish(brake_bool_msg)

    def sub_scan(self):
        sub_scan = rospy.Subscriber("scan",LaserScan,self.scan_callback)
        rospy.spin()
    
    def sub_odom(self):
        sub_odom = rospy.Subscriber("odom",Odometry,self.odom_callback)
        rospy.spin()

    def run(self):
        rospy.init_node('safety_node')
        self.ttc_threshold = rospy.get_param("ttc_threshold")
        self.brake_publisher = rospy.Publisher("brake",AckermannDriveStamped,queue_size=0)
        self.brake_bool_publisher = rospy.Publisher("brake_bool",Bool,queue_size=0)

        self.scan_subscriber.start()
        self.odom_subscriber.start()


if __name__ == '__main__':
    sn = Safety()
    sn.run()