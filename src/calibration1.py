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

x_g = []
y_g = []
z_g = []

def plot_ground_truth():
	f = open('/home/mahindra/tushar/06.txt','r')
	a=[]
	for row in f :
	    a.append([float(x) for x in row.split()])
        for r in a:
            x_g.append(r[3])
	    y_g.append(r[7])
            z_g.append(r[11])

def save_result_text():
	f = open('/home/rrc/catkin_ws/src/viso2-hydro/data/odometry_06.txt' , 'w')
	f.writelines(str(x[i])+" "+str(y[i])+" "+str(z[i])+"\n"for i in range(0,len(x)))
	f.close()

if __name__ == '__main__':
	rospy.init_node("displacement")
	rospy.Subscriber("/stereo_odometer/odometry", Odometry , print_position)
	rospy.Subscriber("/new_vo" ,Odometry, print_position1)
	#rospy.Subscriber("/fix" ,Odometry, print_position2)
							
	rospy.spin()
	plot_ground_truth()
#	save_result_text()
	#pl.plot(z , x, 'r',[-0.3075,1.8896,2.522,3.4,2.832,1.906,-.686] , [-0.0873,-0.5973,-0.36619,-1.3018,-2.098,-2.5384,-1.966] ,'g')
	pl.xlabel('z coordinate')
	pl.ylabel('x coordinate')
	pl.text( 60, .025,'KITTI DATASET \n green-> ground truth , red-> visual odometry')
	#pl.show()
        pl.plot(x_e, z_e,'-',x,z,'-',x_g,z_g,'-')
	pl.show()
	#pl.plot(x,z)
	#pl.show()
	#pl.plot(x_g,y_g)
	#pl.show()
	#pl.plot(latitude,longitude)
	#pl.show()	
