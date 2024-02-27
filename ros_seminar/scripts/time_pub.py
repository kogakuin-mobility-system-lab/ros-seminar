#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

rospy.init_node('time_pub')
pub = rospy.Publisher('UnixTime', Float64 , queue_size=1)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    now = rospy.get_time()
    pub.publish(now)
    rate.sleep()