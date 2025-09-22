#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def cb(msg):
    # copy x,y,yaw từ TEB
    out = Twist()
    out.linear.x = msg.linear.x
    out.linear.y = msg.linear.y
    out.angular.z = msg.angular.z
    # giữ z = 0 (hover, không điều khiển lên xuống)
    out.linear.z = 0.0
    pub.publish(out)

rospy.init_node("cmd_vel_with_alt_hold")
pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
rospy.Subscriber("/cmd_vel", Twist, cb)
rospy.spin()

