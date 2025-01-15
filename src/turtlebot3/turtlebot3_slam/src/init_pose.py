#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# Node initialization
rospy.init_node('init_pose')
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 1)

# Construct message
init_msg = PoseWithCovarianceStamped()
init_msg.header.frame_id = "map"

init_msg.pose.pose.position.x = 0.06336669623851776
init_msg.pose.pose.position.y = 0.3741321563720703
init_msg.pose.pose.orientation.x = 0.0
init_msg.pose.pose.orientation.y = 0.0
init_msg.pose.pose.orientation.z = -0.6986095153990264
init_msg.pose.pose.orientation.w = 0.7155031411488963

# Delay
rospy.sleep(1)

# Publish message
rospy.loginfo("setting initial pose")
pub.publish(init_msg)
rospy.loginfo("initial pose is set")
