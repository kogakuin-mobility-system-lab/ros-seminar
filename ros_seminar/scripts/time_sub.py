#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def callback(message):
    print(message.data)

if __name__ == "__main__":
    rospy.init_node('time_sub')
    sub = rospy.Subscriber('UnixTime', Float64 , callback)
    rospy.spin()