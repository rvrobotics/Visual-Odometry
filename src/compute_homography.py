#!/usr/bin/env python  
'''
This code generates homography matrix using feature corrospondance and
compare it with homography matrices ffor R and T of Visual Odometry.
It also does correction in Visual Odometry and publish it on new topic.

Author:Tushar Vaidya
Contect:mail@tusharvaidya.info
'''
import roslib
import rospy
import math
import tf
import cv2
from cv_bridge import CvBridge,CvBridgeError 
import geometry_msgs.msg
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from message_filters import Subscriber , TimeSynchronizer
import random
from nav_msgs.msg import Odometry
#from common import draw_str

H_tracking = np.array([])
H_roi = np.array([])

R_vo = np.array([])
T_vo = np.array([])

new_vo_pub = rospy.Publisher("/new_vo",Odometry)

def print_position(a):
	
	
	(X, Y, Z) = (a.pose.pose.position.x ,\
			a.pose.pose.position.y ,\
			a.pose.pose.position.z )	
	(x,y,z,w) = (a.pose.pose.orientation.x, \
			a.pose.pose.orientation.y, \
			a.pose.pose.orientation.z, \
			a.pose.pose.orientation.w)
	
	global T_vo
 	T_vo = np.array([X,Y,Z])
	global R_vo
	R_vo = np.array([x,y,z,w])
	

lk_params = dict( winSize  = (19, 19),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict( maxCorners = 1000,
                       qualityLevel = 0.01,
                       minDistance = 8,
                       blockSize = 19 )

def checkedTrace(img0, img1, p0, back_threshold = 1.0):
    p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
    p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
    d = abs(p0-p0r).reshape(-1, 2).max(-1)
    status = d < back_threshold
    return p1, status

green = (0, 255, 0)
red = (0, 0, 255)

p0 = None
use_ransac = True
def callback(l_image, r_image):
	global p0
	bridge = CvBridge()
	l_cv_image = bridge.imgmsg_to_cv2(l_image,"bgr8")
	r_cv_image = bridge.imgmsg_to_cv2(r_image,"bgr8")
	l_cv_image_roi = l_cv_image[250:350 , 500:700]
	frame = l_cv_image_roi.copy()
	frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	vis = frame.copy()
        p0 = cv2.goodFeaturesToTrack(frame_gray, **feature_params)
	if p0 is not None:
 	    p1 = p0
	    gray0 = frame_gray
	    gray1 = frame_gray
	if p0 is not None:
	    p2, trace_status = checkedTrace(gray1, frame_gray, p1)
												     
	    p1 = p2[trace_status].copy()
	    p0 = p0[trace_status].copy()
	    gray1 = frame_gray
												     
	    if len(p0) < 4:
		self.p0 = None
		return
	    global H_tracking
	    H_tracking, status = cv2.findHomography(p0, p1, (0, cv2.RANSAC)[use_ransac], 10.0)
	    #print H
	else:
            p = cv2.goodFeaturesToTrack(frame_gray, **feature_params)
        frame0 = frame.copy()
#        p0 = cv2.goodFeaturesToTrack(frame_gray, **feature_params)
#        if p0 is not None:
# 	    p1 = p0
#	    gray0 = frame_gray
#	    gray1 = frame_gray

def compute_H():
	(x,y,z,w) = R_vo 
	(tx,ty,tz) = T_vo
	R = np.array([[1-y*y , x*y - w*z, x*z+w*y],\
		      [x*y+w*z ,1-(x*x+w*z), y*x-w*x ],\
		      [x*z-w*y, y*z+w*x, -(x*x+y*y)]])
        t = np.array([tx , ty, tz])
	n = np.array([0,1,0])
	d = 0.5
	global H_roi
	H_roi = R + t* n.transpose()/d
	#print H

def correct_rnt(H1,H2,R,T,n):
	T_good =np.array([])
	dt=1.35

	error_min =100
	for i in range(100):
		T_new = random.uniform(0,0.5)
		(x,y,z,w) =  R_vo
		R = np.array([[1-y*y , x*y - w*z, x*z+w*y],\
		      	      [x*y+w*z ,1-(x*x+w*z), y*x-w*x ],\
		      	      [x*z-w*y, y*z+w*x, -(x*x+y*y)]])
		H = R + (T_vo+T_new) * n.transpose() /1.35 
	#	print np.linalg.det(np.subtract(H1,H2))
	#	print  error_min
		if error_min > np.linalg.det(np.subtract(H1,H)):
			T_good = T_new
			error_min =  np.linalg.det(np.subtract(H1,H))
	if not T_good:
		return "NULL"
	else:	
		global new_vo_pub
		s = Odometry()
		s.pose.pose.position.x = T_vo[0] +T_good
		s.pose.pose.position.y = T_vo[1] +T_good
		s.pose.pose.position.z = T_vo[2] +T_good
		s.pose.pose.orientation.x = R_vo[0]
		s.pose.pose.orientation.y = R_vo[1]
		s.pose.pose.orientation.z = R_vo[2]
		s.pose.pose.orientation.w = R_vo[3]
		new_vo_pub.publish(s)
		return T_vo+T_good
		
	
	
	
			
if __name__ == '__main__':
    rospy.init_node('homography')
    listener = tf.TransformListener()
    #rospy.Subscriber("/surface_normals",PointCloud2,callback)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
       # try:
       #     (trans,rot) = listener.lookupTransform('/start', '/kitti_stereo', rospy.Time(0))
       # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
       # 	continue
	
	rospy.Subscriber("/stereo_odometer/odometry", Odometry , print_position)
	tss = TimeSynchronizer([Subscriber("/kitti_stereo/left/image_rect", Image),Subscriber("/kitti_stereo/right/image_rect", Image)],10)
	tss.registerCallback(callback)
	if T_vo.any():
		compute_H()
	#print H_tracking
	#print("\n")
	#print H_roi
	#print("\n")
	if H_tracking.any():
		print correct_rnt(H_tracking, H_roi, R_vo, T_vo ,np.array([0,1,0]))
	rate.sleep()
