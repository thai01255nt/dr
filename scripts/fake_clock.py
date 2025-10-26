#!/usr/bin/env python3
import rospy
from rosgraph_msgs.msg import Clock
import airsim
import time

rospy.init_node("airsim_clock_publisher")
client = airsim.MultirotorClient()
client.confirmConnection()
pub = rospy.Publisher("/clock", Clock, queue_size=1)

while not rospy.is_shutdown():
    # Lấy IMU làm nguồn thời gian vì tốc độ cao hơn camera
    imu = client.getImuData()
    ts = imu.time_stamp  # nanoseconds
    secs = ts // 1_000_000_000
    nsecs = ts % 1_000_000_000
    sim_time = rospy.Time(secs, nsecs)
    pub.publish(Clock(clock=sim_time))
    time.sleep(0.002)  # 100Hz, dùng real-time sleep
