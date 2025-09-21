import rospy
from geometry_msgs.msg import Twist
def send_velocity():
  rospy.init_node('send_vel_2d')
  pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
  rate = rospy.Rate(10)  # 10 Hz
  vel = Twist()
  vel.linear.x = 0.0  # forward ENU
  vel.linear.y = 2.0  # right ENU
  vel.linear.z = 0.0  # giữ cao độ
  vel.angular.x = 0.0
  vel.angular.y = 0.0
  vel.angular.z = 0.0  # không quay
  while not rospy.is_shutdown():
    print("Sending velocity...")
    pub.publish(vel)
    rate.sleep()

send_velocity()
