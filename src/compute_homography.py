#!/usr/bin/env python  
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
#from common import draw_str

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
	    H, status = cv2.findHomography(p0, p1, (0, cv2.RANSAC)[use_ransac], 10.0)
	    print H
	else:
            p = cv2.goodFeaturesToTrack(frame_gray, **feature_params)
        frame0 = frame.copy()
#        p0 = cv2.goodFeaturesToTrack(frame_gray, **feature_params)
#        if p0 is not None:
# 	    p1 = p0
#	    gray0 = frame_gray
#	    gray1 = frame_gray

def compute_H():
	(x,y,z,w) = rot
	(tx,ty,tz) = trans
	R = np.array([[1-y*y , x*y - w*z, x*z+w*y],\
		      [x*y+w*z ,1-(x*x+w*z), y*x-w*x ],\
		      [x*z-w*y, y*z+w*x, -(x*x+y*y)]])
        t = np.array([tx , ty, tz])
	n = np.array([0,1,0])
	d = 0.5
	H = R + t* n.transpose()/d
	#print H

if __name__ == '__main__':
    rospy.init_node('homography')

    listener = tf.TransformListener()

    #rospy.Subscriber("/surface_normals",PointCloud2,callback)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/start', '/kitti_stereo', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        	continue
	tss = TimeSynchronizer([Subscriber("/kitti_stereo/left/image_rect", Image),Subscriber("/kitti_stereo/right/image_rect", Image)],10)
	tss.registerCallback(callback)
	compute_H()
	#print("\n")
	rate.sleep()
