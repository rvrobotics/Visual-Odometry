#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2.cv as cv
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
#import pcl

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("Points",PointCloud)

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/kitti_stereo/left/image_rect",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      #cv_image_roi = cv_image[250:350 , 500:700]
    except CvBridgeError, e:
      print e

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
        cv2.rectangle(cv_image, (100+400,100+150), (200+500,200+150),255)
	#cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    #cv_image_roi_disp = self.getDisparity(np.array(cv_image) , np.array(cv_image)) 
    
    gray_left = cv2.cvtColor(cv_image, cv.CV_BGR2GRAY)
    gray_right = cv2.cvtColor(cv_image, cv.CV_BGR2GRAY)
    #print gray_left.shape
    c, r = gray_left.shape
    sbm = cv.CreateStereoBMState()
    disparity = cv.CreateMat(c, r, cv.CV_32F)
    sbm.SADWindowSize = 9
    sbm.preFilterType = 1
    sbm.preFilterSize = 5
    sbm.preFilterCap = 61
    sbm.minDisparity = -39
    sbm.numberOfDisparities = 112
    sbm.textureThreshold = 507
    sbm.uniquenessRatio= 0
    sbm.speckleRange = 8
    sbm.speckleWindowSize = 0
									    
    gray_left = cv.fromarray(gray_left)
    gray_right = cv.fromarray(gray_right)
									    
    cv.FindStereoCorrespondenceBM(gray_left, gray_right, disparity, sbm)
    disparity_visual = cv.CreateMat(c, r, cv.CV_8U)
    cv.Normalize(disparity, disparity_visual, 0, 255, cv.CV_MINMAX)
    disparity_visual = np.array(disparity_visual)                       
    cv2.imshow("Image window ROI", np.array(disparity_visual[250:350 , 500:700] ))
    #disparity = cv.fromarray(disparity)
    Q = [[1,
          0,
          0,
          -254.903519],
          [0,
          1,
          0,
          -201.89949],
          [0,
          0,
          0,
          389.956085],
          [0,
          0,
          -0.1134097419840026,
          4.165856884426882]]
    dest = cv.CreateMat(4, 4, cv.CV_32FC1)
    src = cv.fromarray(np.array(Q))
    cv.Convert(src, dest)
    
    point_cloud = cv.CreateImage( (200,100), IPL_DEPTH_32F, 1)
    point_cloud = cv2.reprojectImageTo3D(np.array(disparity[250:350 , 500:700]),Q=np.array(Q))
    pts = PointCloud()
    pt= Point()
    #pts_pcl = pcl.PointCloud()
    for i in range(0,200):
	   for j in range(0 , 100):
		    x, y, z = point_cloud[j][i]
		    pt.x =x
		    pt.y =y
		    pt.z =z
    		    pts.points.append(pt)
    pts.header.frame_id= 'start'
    #pts_pcl.from_array(np.array(point_cloud,dtype= np.float32))

    try:
     self.image_pub.publish(pts)
    except CvBridgeError, e:
     print e
    cv2.waitKey(3)

    #try:
     # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError, e:
     # print e

  def getDisparity(imgLeft, imgRight, method="BM"):
    gray_left = cv2.cvtColor(imgLeft, cv.CV_BGR2GRAY)
    gray_right = cv2.cvtColor(imgRight, cv.CV_BGR2GRAY)
    #print gray_left.shape
    c, r = gray_left.shape
    if method == "BM":
	sbm = cv.CreateStereoBMState()
	disparity = cv.CreateMat(c, r, cv.CV_32F)
	sbm.SADWindowSize = 9
	sbm.preFilterType = 1
	sbm.preFilterSize = 5
	sbm.preFilterCap = 61
	sbm.minDisparity = -39
	sbm.numberOfDisparities = 112
	sbm.textureThreshold = 507
	sbm.uniquenessRatio= 0
	sbm.speckleRange = 8
	sbm.speckleWindowSize = 0

	gray_left = cv.fromarray(gray_left)
	gray_right = cv.fromarray(gray_right)

	cv.FindStereoCorrespondenceBM(gray_left, gray_right, disparity, sbm)
	disparity_visual = cv.CreateMat(c, r, cv.CV_8U)
	cv.Normalize(disparity, disparity_visual, 0, 255, cv.CV_MINMAX)
	disparity_visual = np.array(disparity_visual)                        

    elif method == "SGBM":
	sbm = cv2.StereoSGBM()
	sbm.SADWindowSize = 9;
	sbm.numberOfDisparities = 96;
	sbm.preFilterCap = 63;
	sbm.minDisparity = -21;
	sbm.uniquenessRatio = 7;
	sbm.speckleWindowSize = 0;
	sbm.speckleRange = 8;
	sbm.disp12MaxDiff = 1;
	sbm.fullDP = False;

	disparity = sbm.compute(gray_left, gray_right)
	disparity_visual = cv2.normalize(disparity, alpha=0, beta=255, norm_type=cv2.cv.CV_MINMAX, dtype=cv2.cv.CV_8U)

    return disparity_visual

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
