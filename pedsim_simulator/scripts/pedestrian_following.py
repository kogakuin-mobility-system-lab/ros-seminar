#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from pedsim_msgs.msg import AgentStates
from nav_msgs.msg import Odometry
import numpy as np
import tf
from tf.transformations import euler_from_quaternion

ped_pos = [0.0, 0.0]
robot_pose = [0.0, 0.0, 0.0]

def subscribe_pedestrian_position(message):
    global ped_pos
    # print(message.agent_states[0].pose.position)
    ped_pos[0] = message.agent_states[0].pose.position.x
    ped_pos[1] = message.agent_states[0].pose.position.y
    # ped_z = message.agent_states[0].pose.position.z

    # print(ped_x, ped_y)

def subscribe_robot_position(message):
    global robot_pose
    # print(message.pose.pose)

    robot_pose[0] = message.pose.pose.position.x
    robot_pose[1] = message.pose.pose.position.y

    qx = message.pose.pose.orientation.x
    qy = message.pose.pose.orientation.y
    qz = message.pose.pose.orientation.z
    qw = message.pose.pose.orientation.w
    q = (qx, qy, qz, qw)

    e = euler_from_quaternion(q)
    robot_pose[2] = e[2]
    
    # print(robot_pose[2])

def following():
    rate = rospy.Rate(10)
    cmd = Twist()
    while not rospy.is_shutdown():
        # print(ped_pos)

        vec = np.array(ped_pos) - np.array([robot_pose[0], robot_pose[1]])
        dist = np.linalg.norm(vec)
        ang = np.arctan2(vec[1], vec[0]) - robot_pose[2]

        while ang > np.pi: ang -= 2*np.pi
        while ang < -np.pi: ang += 2*np.pi

        if dist > 3.0:
            cmd.linear.x = 1.0
        else:
            cmd.linear.x = 0.0

        if ang > 0.1:
            cmd.angular.z = 1.0
        elif ang < -0.1:
            cmd.angular.z = -1.0
        else:
            cmd.angular.z = 0.0

        print(dist, ang, cmd.linear.x, cmd.angular.z)

        pub.publish(cmd)
        rate.sleep()
    

if __name__ == "__main__":
    rospy.init_node('pedestrian_following')
    sub1 = rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates , subscribe_pedestrian_position)
    sub2 = rospy.Subscriber('/pedsim_simulator/robot_position', Odometry , subscribe_robot_position)
    pub = rospy.Publisher('/pedbot/control/cmd_vel', Twist, queue_size=1)
    following()
    rospy.spin()
