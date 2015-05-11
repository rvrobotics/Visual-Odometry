import numpy as np
import cv2 , video
from common import draw_str
import rospy

if __name__ == '__main__':
	rospy.init_node("homography_from_feature_tracking")
	rospy.Subscriber("/kitti_stereo/left/image_rect",)
