#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, tf, roslib
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry

pose = Pose()

def receivePosition(actualPose):
        global pose
        pose = actualPose
        br = tf.TransformBroadcaster()
        rate = rospy.rate(10.00)
        br.sendTransform((pose.position.x,pose.position.y.pose.z),(0,0,0),rospy.Time.now(),'odom','world')

if __name__ == 'main':
        rospy.init_node('competitior')
        rospy.Subscriber('/odom',Pose,receivePosition)
	rospy.spin()
