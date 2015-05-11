#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2, tf
import numpy as np
from numpy import deg2rad, cos, sin
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan, Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import matplotlib.pyplot as pl

x = []
y = []
z = []
def print_position(a):
	rospy.loginfo(a.pose.pose.position)
	#global pose
	x.append(a.pose.pose.position.x)
	y.append(a.pose.pose.position.y)
	z.append(a.pose.pose.position.z)

x_e = []
y_e = []
z_e = []

def print_position1(a):
	x_e.append(a.pose.pose.position.x)
	y_e.append(a.pose.pose.position.y)
	z_e.append(a.pose.pose.position.z)

latitude  = []
longitude = []
altitude  = []

def print_position2(a):
	latitude.append(a.latitude)
	longitude.append(a.longitude)
	altitude.append(a.altitude)

if __name__ == '__main__':
	rospy.init_node("displacement")
	rospy.Subscriber("/stereo_odometer/odometry", Odometry , print_position)
	# rospy.Subscriber("/encoder" ,Odometry, print_position1)
	#rospy.Subscriber("/fix" ,Odometry, print_position2)
							
	rospy.spin()
	pl.plot(x , z)
	pl.show()
	# pl.plot(x_e, y_e)
	# pl.show()
	#pl.plot(latitude,longitude)
	#pl.show()	
