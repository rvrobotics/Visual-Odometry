#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2, tf
import numpy as np
from numpy import deg2rad
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


radStep = deg2rad(0)

pose = Pose()

t = Twist()
# Position callback
def receivePosition(actualPose):
    global pose
    pose = actualPose
    #publishing topics
    pubVel   = rospy.Publisher('/husky/cmd_vel', Twist)
    global t
    t.linear.x=0.5
    t.linear.y = 0.0
    t.linear.z = 0.0
    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = 0
    pubVel.publish(t)
    if(pose.pose.pose.position.x > 5.0):
        t.linear.x=0.0
        pubVel.publish(t)

if __name__ == '__main__':
    # Initialize client node
    rospy.init_node('robot_mover')
    rospy.Subscriber("/odom", Odometry, receivePosition)
    
    rospy.spin()
