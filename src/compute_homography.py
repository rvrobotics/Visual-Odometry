#!/usr/bin/env python  
'''
This code generates homography matrix using feature corrospondance 
using SIFT feature and knn tracking and compare it with homography 
matrices for R and T of Visual Odometry from viso2 algorithm .
It also does correction in Visual Odometry and publish it on new topic.

Author:Tushar Vaidya
Contact:tushar5610@gmail.com
'''
import roslib
import rospy
import math
import tf
import cv2
import cv
from cv_bridge import CvBridge,CvBridgeError 
import geometry_msgs.msg
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from message_filters import Subscriber , TimeSynchronizer
import random
from nav_msgs.msg import Odometry
from find_obj import filter_matches,explore_match
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

p0 = np.array([])
use_ransac = True
p1 = None
#g_image = cv2.Mat()
def callback(l_image, r_image):
	global p0
	global p1
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
def drawMatches(img1, kp1, img2, kp2, matches):
    """
    My own implementation of cv2.drawMatches as OpenCV 2.4.9
    does not have this function available but it's supported in
    OpenCV 3.0.0

    This function takes in two images with their associated 
    keypoints, as well as a list of DMatch data structure (matches) 
    that contains which keypoints matched in which images.

    An image will be produced where a montage is shown with
    the first image followed by the second image beside it.

    Keypoints are delineated with circles, while lines are connected
    between matching keypoints.

    img1,img2 - Grayscale images
    kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint 
              detection algorithms
    matches - A list of matches of corresponding keypoints through any
              OpenCV keypoint matching algorithm
    """

    # Create a new output image that concatenates the two images together
    # (a.k.a) a montage
    rows1 = img1.shape[0]
    cols1 = img1.shape[1]
    rows2 = img2.shape[0]
    cols2 = img2.shape[1]

    out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

    # Place the first image to the left
    out[:rows1,:cols1,:] = np.dstack([img1, img1, img1])

    # Place the next image to the right of it
    out[:rows2,cols1:cols1+cols2,:] = np.dstack([img2, img2, img2])

    # For each pair of points we have between both images
    # draw circles, then connect a line between them
    for mat in matches:

        # Get the matching keypoints for each of the images
        img1_idx = mat.queryIdx
        img2_idx = mat.trainIdx

        # x - columns
        # y - rows
        (x1,y1) = kp1[img1_idx].pt
        (x2,y2) = kp2[img2_idx].pt

        # Draw a small circle at both co-ordinates
        # radius 4
        # colour blue
        # thickness = 1
        cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)   
        cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)

        # Draw a line in between the two points
        # thickness = 1
        # colour blue
        cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255, 0, 0), 1)


    # Show the image
    cv2.imshow('Matched Features', out)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

image_0 = np.array([])
def SIFT_callback(l_image, r_image):
	global image_0
	global p0
	global p1
	bridge = CvBridge()
	l_cv_image = bridge.imgmsg_to_cv2(l_image,"bgr8")
	#l_cv_image_roi = l_cv_image[250:350 , 500:700]
        #r_cv_image = bridge.imgmsg_to_cv2(r_image,"bgr8")
	if image_0.any():
		image_1 = l_cv_image.copy()
	else :
          	image_0 = l_cv_image.copy()
		return

	sift = cv2.SIFT()
	mask = np.zeros(image_1.shape[0:2],dtype=np.uint8)
	
	mask[250:351,500:701]=1	
        #msk = cv.fromarray(mask,cv2.CV_8UC1)
	kp0 , ds0 = sift.detectAndCompute(image_0,mask)
	kp1, ds1 = sift.detectAndCompute(image_1,mask)

	FLYNN_INDEX_KDTREE = 0   	
	index_params = dict(algorithm= FLYNN_INDEX_KDTREE,
				trees=5)
				#key_size=12,
				#multi_probe_level =1)
	search_params = dict(checks = 1)

	flann = cv2.FlannBasedMatcher(index_params,search_params)
	matches = flann.knnMatch(ds0,ds1,k=2)
	#bf = cv2.BFMatcher()
	#matches = bf.knnMatch(ds0,ds1,k=0)
	good = []
	for m,n in matches:
		if m.distance < 0.7*n.distance:
			good.append(m)
	
	
	p0 = np.float32([kp0[m.queryIdx].pt for m in good]).reshape(-1,1,2)
	p1 = np.float32([kp1[m.trainIdx].pt for m in good]).reshape(-1,1,2)
	explore_match('features',image_0,image_1,good)
	#for a,b in zip(p_0,p_1):
	#	x0,y0=a[0]
  	#	x1,y1=b[0]
	#	if 250<x0 and x0<350 and 500<y0 and y0<700 and 250<x1 and x1<350 and 500<y1 and y1<700:
	#		p0.push([x0,y0])
	#		p1.push([x1,y1])
	image_0 = image_1.copy()
	#if p0.any() and p1.any():
	#print p0.size 
	#print p1.size
	#drawMatches(image_0[250:351,500:701],kp0,image_1[250:351,500:701],kp1,matches)	

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
		T_new = random.uniform(-0.5,0.5)
		(x,y,z,w) =  R_vo
		R = np.array([[1-y*y , x*y - w*z, x*z+w*y],\
		      	      [x*y+w*z ,1-(x*x+w*z), y*x-w*x ],\
		      	      [x*z-w*y, y*z+w*x, -(x*x+y*y)]])
		H = R + (T_vo+T_new) * n.transpose() / dt 
	#	print np.linalg.det(np.subtract(H1,H2))
	#	print  error_min
		if error_min > np.linalg.det(np.subtract(H1,H)): #np.linalg.norm(H1 - H,'fro'):
			T_good = T_new
			error_min =  np.linalg.det(np.subtract(H1,H)) #np.linalg.norm(H1 - H,'fro')
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
		
	
def correct_rnt_new(H1,H2,R,T,n):
	T_good =np.array([])
	dt=1.35

	error_min =100000
	(x,y,z,w) =  R_vo
	R = np.array([[1-y*y , x*y - w*z, x*z+w*y],\
      		      [x*y+w*z ,1-(x*x+w*z), y*x-w*x ],\
      		      [x*z-w*y, y*z+w*x, -(x*x+y*y)]])
	H = R + T_vo * n.transpose() / dt 
	for p,q in zip(p0,p1):
		X_old = np.transpose(np.array(p[0]))
		X_old = np.append(X_old,1)
        	X_new = np.transpose(np.array(q[0]))
		X_new = np.append(X_new,1)
		error_min = error_min + np.linalg.norm(H * X_old - X_new, 'fro') 
	#print error_min
	for i in range(100):
		error = 0;
		T_new_x = random.uniform(-1.5,1.5)
		T_new_z = random.uniform(-1.5,1.5)
		T_new = [T_vo[0]+T_new_x,T_vo[1],T_vo[2]+T_new_z]
		#(x,y,z,w) =  R_vo
		#R = np.array([[1-y*y , x*y - w*z, x*z+w*y],\
	      	#	      [x*y+w*z ,1-(x*x+w*z), y*x-w*x ],\
	      	#	      [x*z-w*y, y*z+w*x, -(x*x+y*y)]])
		#H = R + (T_vo+T_new) * n.transpose() / dt 
		
		H = R + T_new * n.transpose() / dt 
		#	print np.linalg.det(np.subtract(H1,H2))
		#	print  error_min
	
		for p,q in zip(p0,p1):
			X_old = np.transpose(np.array(p[0]))
			X_old = np.append(X_old,1)
               		X_new = np.transpose(np.array(q[0]))
			X_new = np.append(X_new,1)
			error = error + np.linalg.norm(H * X_old - X_new, 'fro') 
		if error < error_min: 
			T_good = T_new
			error_min = error 
	#print error_min
	#print "\n" 
	if not T_good:
		return "NULL"
	else:	
		global new_vo_pub
		s = Odometry()
		s.pose.pose.position.x = T_good[0]
		s.pose.pose.position.y = T_good[1]
		s.pose.pose.position.z = T_good[2]
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
        try:
            (trans,rot) = listener.lookupTransform('/start', '/kitti_stereo', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        	continue
	
	rospy.Subscriber("/stereo_odometer/odometry", Odometry , print_position)
	tss = TimeSynchronizer([Subscriber("/kitti_stereo/left/image_rect", Image),Subscriber("/kitti_stereo/right/image_rect", Image)],10)
	tss.registerCallback(SIFT_callback)
	if T_vo.any():
		compute_H()
	#print H_tracking
	#print("\n")
	#print H_roi
	#print("\n")
	if p0.any():
		correct_rnt_new(H_tracking, H_roi, R_vo, T_vo ,np.array([0,1,0]))
	rate.sleep()
