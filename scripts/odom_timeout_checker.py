#!/usr/bin/env python3
"""
Odometry Timeout Checker with Covariance Scaling

Monitors orb_slam3/odom and publishes to odometry/visual for Cartographer:
- When odometry is fresh → forward with original covariance (map -> camera_link)
- When timeout detected → publish with VERY HIGH covariance
- Cartographer will automatically reduce trust for high-covariance odometry
- All frames: map -> camera_link (TF transforms to base_link)

Usage:
    rosrun cart_teb_test odom_timeout_checker.py
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class OdomTimeoutChecker:
    def __init__(self):
        rospy.init_node('odom_timeout_checker', anonymous=False)

        # Parameters
        self.timeout = rospy.get_param('~timeout', 0.5)  # 500ms timeout
        self.publish_rate = rospy.get_param('~publish_rate', 20)  # Hz
        self.covariance_scale_factor = rospy.get_param('~covariance_scale', 1000.0)  # 1000x increase

        # Topics
        self.input_topic = rospy.get_param('~input_topic', '/orb_slam3/odom')
        self.output_topic = rospy.get_param('~output_topic', '/odometry/visual')

        # State
        self.last_odom = None
        self.last_odom_time = None
        self.is_timeout = False
        self.timeout_start_time = None

        # Publisher & Subscriber
        self.odom_pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.input_topic, Odometry, self.odom_callback)

        rospy.loginfo("[OdomTimeoutChecker] Started")
        rospy.loginfo(f"  Input: {self.input_topic}")
        rospy.loginfo(f"  Output: {self.output_topic}")
        rospy.loginfo(f"  Timeout: {self.timeout}s")
        rospy.loginfo(f"  Covariance scale (on timeout): {self.covariance_scale_factor}x")

        # Timer for publishing
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)

    def odom_callback(self, msg):
        """Callback when new odometry is received"""
        self.last_odom = msg
        self.last_odom_time = rospy.Time.now()

        # Check if recovering from timeout
        if self.is_timeout:
            duration = (rospy.Time.now() - self.timeout_start_time).to_sec()
            rospy.loginfo(f"[OdomTimeoutChecker] ✓ Odometry recovered after {duration:.2f}s timeout")
            self.is_timeout = False
            self.timeout_start_time = None

    def scale_covariance(self, original_cov, scale_factor):
        """
        Scale covariance matrix by a factor

        Args:
            original_cov: Original 36-element covariance array
            scale_factor: Multiplier for covariance values

        Returns:
            Scaled covariance array
        """
        cov_array = np.array(original_cov).reshape(6, 6)

        # Only scale diagonal elements (variances)
        # Off-diagonal (covariances) are typically 0 anyway
        for i in range(6):
            cov_array[i, i] *= scale_factor

        return cov_array.flatten().tolist()

    def create_high_uncertainty_odom(self, base_odom):
        """
        Create odometry with extremely high covariance

        Tells Cartographer: "I have NO IDEA where the camera is!"
        Frame: map -> camera_link (Cartographer will transform to base_link via TF)
        """
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = rospy.Time.now()
        # Always use map frame (camera tracking in map)
        odom.header.frame_id = "map"
        # Always use camera_link as child frame
        odom.child_frame_id = "camera_link"

        if base_odom:
            # Use last known pose but with HUGE uncertainty
            odom.pose.pose = base_odom.pose.pose
            odom.twist.twist = base_odom.twist.twist

            # Scale covariances by scale_factor
            odom.pose.covariance = self.scale_covariance(
                base_odom.pose.covariance,
                self.covariance_scale_factor
            )
            odom.twist.covariance = self.scale_covariance(
                base_odom.twist.covariance,
                self.covariance_scale_factor
            )
        else:
            # No previous odometry - create zero pose with max uncertainty
            odom.pose.pose.position.x = 0.0
            odom.pose.pose.position.y = 0.0
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.w = 1.0

            # Maximum uncertainty (1000m std in position!)
            max_variance = 1000000.0  # (1000m)^2
            pose_cov = np.zeros(36)
            pose_cov[0] = max_variance   # x
            pose_cov[7] = max_variance   # y
            pose_cov[14] = max_variance  # z
            pose_cov[21] = 100.0         # roll
            pose_cov[28] = 100.0         # pitch
            pose_cov[35] = 100.0         # yaw
            odom.pose.covariance = pose_cov.tolist()

            twist_cov = np.zeros(36)
            twist_cov[0] = 100.0   # vx
            twist_cov[7] = 100.0   # vy
            twist_cov[14] = 100.0  # vz
            odom.twist.covariance = twist_cov.tolist()

        return odom

    def timer_callback(self, event):
        """Periodic timer to check timeout and publish odometry"""
        now = rospy.Time.now()

        # Check for timeout
        if self.last_odom_time is not None:
            elapsed = (now - self.last_odom_time).to_sec()

            if elapsed > self.timeout:
                # TIMEOUT!
                if not self.is_timeout:
                    rospy.logwarn(f"[OdomTimeoutChecker] ⚠ Odometry TIMEOUT! No data for {elapsed:.2f}s")
                    rospy.logwarn("[OdomTimeoutChecker] → Publishing HIGH covariance odometry")
                    self.is_timeout = True
                    self.timeout_start_time = now

                # Publish high-uncertainty odometry
                timeout_odom = self.create_high_uncertainty_odom(self.last_odom)
                self.odom_pub.publish(timeout_odom)
            else:
                # Normal operation - forward odometry as-is
                if self.last_odom is not None:
                    # Update timestamp to current time
                    self.last_odom.header.stamp = now
                    self.odom_pub.publish(self.last_odom)
        else:
            # No odometry received yet
            if not self.is_timeout:
                rospy.logwarn("[OdomTimeoutChecker] Waiting for initial odometry...")
                self.is_timeout = True
                self.timeout_start_time = now

            # Publish zero odometry with max uncertainty
            timeout_odom = self.create_high_uncertainty_odom(None)
            self.odom_pub.publish(timeout_odom)

    def run(self):
        """Main loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        checker = OdomTimeoutChecker()
        checker.run()
    except rospy.ROSInterruptException:
        pass
