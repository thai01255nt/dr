##!/usr/bin/env python3
#import rospy
#from geometry_msgs.msg import PoseWithCovarianceStamped
#
#rospy.init_node('init_pose_pub')
#pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
#
#rospy.sleep(30.0)
#msg = PoseWithCovarianceStamped()
#msg.header.frame_id = "map"
#msg.header.stamp = rospy.Time.now()
#msg.pose.pose.position.x = 0.0
#msg.pose.pose.position.y = 0.0
#msg.pose.pose.position.z = 0.0
#msg.pose.pose.orientation.x = 0.0
#msg.pose.pose.orientation.y = 0.0
#msg.pose.pose.orientation.z = 0.0
#msg.pose.pose.orientation.w = 1.0
#msg.pose.covariance = [0.0]*36
#
#rospy.sleep(1)  # chờ publisher khởi tạo
#pub.publish(msg)
#rospy.loginfo("Initial pose published")

