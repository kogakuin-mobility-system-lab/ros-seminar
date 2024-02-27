#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    print(len(data.ranges))

if __name__ == "__main__":
    rospy.init_node('scan_sub')
    sub = rospy.Subscriber('scan', LaserScan , callback)
    rospy.spin()
