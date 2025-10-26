#!/usr/bin/env python3
"""
Simple Odometry Forwarder with Timeout Detection

Forwards odometry from orb_slam3/odom to odometry/visual for Cartographer.
Keeps the original frames (map -> camera_link).

When timeout is detected (no messages received), publishes zero odometry with very high uncertainty.

Usage:
    rosrun cart_teb_test odom_forwarder.py
"""

import rospy
from nav_msgs.msg import Odometry
import threading

class OdomForwarder:
    def __init__(self):
        rospy.init_node('odom_forwarder', anonymous=False)

        # Parameters
        self.input_topic = rospy.get_param('~input_topic', '/vins_estimator/odometry')
        self.output_topic = rospy.get_param('~output_topic', '/odometry/visual')
        self.update_timestamp = rospy.get_param('~update_timestamp', False)
        # Don't override frames - keep original (map -> camera_link)
        self.update_frame_id = rospy.get_param('~update_frame_id', False)

        # Timeout checking parameters
        self.is_check_timeout = rospy.get_param('~is_check_timeout', True)
        self.timeout_duration = rospy.get_param('~timeout_duration', 1.0)  # seconds
        self.publish_rate_on_timeout = rospy.get_param('~publish_rate_on_timeout', 10.0)  # Hz

        # Statistics
        self.msg_count = 0
        self.last_log_time = rospy.Time.now()
        self.log_interval = rospy.get_param('~log_interval', 5.0)  # Log every 5s

        # Timeout tracking
        self.last_msg_time = rospy.Time.now()
        self.is_timeout = False
        self.last_received_odom = None
        self.last_timestamp = rospy.Time(0)  # Track last published timestamp
        self.dropped_msg_count = 0

        # Thread safety lock for timestamp and publish operations
        self.lock = threading.Lock()

        # Publisher & Subscriber
        self.odom_pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.input_topic, Odometry, self.odom_callback)

        rospy.loginfo("[OdomForwarder] Started")
        rospy.loginfo(f"  Input:  {self.input_topic}")
        rospy.loginfo(f"  Output: {self.output_topic}")
        rospy.loginfo(f"  Update timestamp: {self.update_timestamp}")
        rospy.loginfo(f"  Keeps original frames: map -> camera_link")
        rospy.loginfo(f"  Timeout checking: {self.is_check_timeout}")
        if self.is_check_timeout:
            rospy.loginfo(f"  Timeout duration: {self.timeout_duration}s")
            rospy.loginfo(f"  Publish rate on timeout: {self.publish_rate_on_timeout} Hz")

    def odom_callback(self, msg):
        """Forward odometry message from orb_slam3/odom to odometry/visual"""
        # Update last message time (outside lock - this is just for timeout detection)
        self.last_msg_time = rospy.Time.now()

        # Store last received odometry (for frame info)
        self.last_received_odom = msg

        # If we were in timeout state, log recovery
        if self.is_timeout:
            rospy.loginfo("[OdomForwarder] Odometry recovered from timeout!")
            self.is_timeout = False

        # Acquire lock for timestamp checking and publishing
        with self.lock:
            # Check if timestamp is newer than last published timestamp
            incoming_timestamp = msg.header.stamp
            if incoming_timestamp <= self.last_timestamp:
                self.dropped_msg_count += 1
                rospy.logwarn_throttle(1.0,
                    f"[OdomForwarder] Dropping message with old/equal timestamp. "
                    f"Incoming: {incoming_timestamp.to_sec():.6f}, "
                    f"Last: {self.last_timestamp.to_sec():.6f} "
                    f"(dropped {self.dropped_msg_count} msgs so far)")
                return

            # Update frame IDs
            msg.header.frame_id = "map"
            msg.child_frame_id = "camera_link"

            # Update timestamp if requested (usually False)
            if self.update_timestamp:
                msg.header.stamp = rospy.Time.now()

            # Publish to Cartographer's visual odometry topic
            self.odom_pub.publish(msg)

            # Update last published timestamp
            self.last_timestamp = incoming_timestamp

        # Statistics (outside lock - not critical)
        self.msg_count += 1

        # Log periodically
        now = rospy.Time.now()
        if (now - self.last_log_time).to_sec() >= self.log_interval:
            rate = self.msg_count / self.log_interval
            rospy.loginfo(f"[OdomForwarder] Forwarding at {rate:.1f} Hz ({self.msg_count} msgs, {self.dropped_msg_count} dropped)")
            self.msg_count = 0
            self.last_log_time = now

    def create_zero_odom_with_high_uncertainty(self):
        """Create a zero odometry message with very high uncertainty"""
        msg = Odometry()

        # Use frames from last received message, or defaults
        if self.last_received_odom is not None:
            msg.header.frame_id = self.last_received_odom.header.frame_id
            msg.child_frame_id = self.last_received_odom.child_frame_id
        else:
            msg.header.frame_id = "map"
            msg.child_frame_id = "camera_link"

        # Use current time for timeout messages
        msg.header.stamp = rospy.Time.now()

        # Zero pose
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # Very high uncertainty (practically infinite)
        # Covariance matrix is 6x6 (x, y, z, roll, pitch, yaw)
        # Set diagonal elements to very large values (1e9 = essentially infinite uncertainty)
        high_uncertainty = 1e9
        msg.pose.covariance = [
            high_uncertainty, 0, 0, 0, 0, 0,  # x variance
            0, high_uncertainty, 0, 0, 0, 0,  # y variance
            0, 0, high_uncertainty, 0, 0, 0,  # z variance
            0, 0, 0, high_uncertainty, 0, 0,  # roll variance
            0, 0, 0, 0, high_uncertainty, 0,  # pitch variance
            0, 0, 0, 0, 0, high_uncertainty   # yaw variance
        ]

        # Zero twist
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0

        # Very high uncertainty for twist as well
        msg.twist.covariance = [
            high_uncertainty, 0, 0, 0, 0, 0,
            0, high_uncertainty, 0, 0, 0, 0,
            0, 0, high_uncertainty, 0, 0, 0,
            0, 0, 0, high_uncertainty, 0, 0,
            0, 0, 0, 0, high_uncertainty, 0,
            0, 0, 0, 0, 0, high_uncertainty
        ]

        return msg

    def check_timeout(self):
        """Check if odometry has timed out and publish zero odom with high uncertainty if so"""
        if not self.is_check_timeout:
            return

        now = rospy.Time.now()
        time_since_last_msg = (now - self.last_msg_time).to_sec()

        if time_since_last_msg > self.timeout_duration:
            if not self.is_timeout:
                rospy.logwarn(f"[OdomForwarder] Odometry TIMEOUT detected! No messages for {time_since_last_msg:.2f}s")
                rospy.logwarn("[OdomForwarder] Publishing zero odometry with very high uncertainty")
                self.is_timeout = True

            # Create zero odometry with very high uncertainty
            zero_odom = self.create_zero_odom_with_high_uncertainty()

            # Acquire lock for timestamp checking and publishing
            with self.lock:
                # Check if timestamp is newer than last published timestamp
                timeout_timestamp = zero_odom.header.stamp
                if timeout_timestamp > self.last_timestamp:
                    # Publish zero odometry
                    self.odom_pub.publish(zero_odom)
                    # Update last published timestamp
                    self.last_timestamp = timeout_timestamp
                # If timeout message has old timestamp, skip it silently (don't spam warnings)

    def run(self):
        """Main loop"""
        if self.is_check_timeout:
            # Run with timer to check for timeouts
            rate = rospy.Rate(self.publish_rate_on_timeout)
            while not rospy.is_shutdown():
                self.check_timeout()
                rate.sleep()
        else:
            # Just spin without timeout checking
            rospy.spin()

if __name__ == '__main__':
    try:
        forwarder = OdomForwarder()
        forwarder.run()
    except rospy.ROSInterruptException:
        pass
