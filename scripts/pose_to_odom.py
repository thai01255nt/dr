#!/usr/bin/env python3
"""
PoseStamped to Odometry Converter with Timeout Handling

Converts ORB-SLAM3 camera_pose (geometry_msgs/PoseStamped)
to odometry (nav_msgs/Odometry) with velocity estimation.

Features:
- Converts pose to odom (map -> camera_link)
- Skips invalid poses (all zeros or at origin)
- Skips first pose after recovery for accurate velocity
- Timeout handling: publishes high-covariance uncertain odom when no valid pose

Usage:
    rosrun cart_teb_test pose_to_odom.py
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PoseToOdom:
    def __init__(self):
        rospy.init_node('pose_to_odom', anonymous=False)

        # Parameters
        self.input_topic = rospy.get_param('~input_topic', '/orb_slam3/camera_pose')
        self.output_topic = rospy.get_param('~output_topic', '/orb_slam3/odom')
        self.use_original_frame = rospy.get_param('~use_original_frame', True)  # Keep camera frame
        self.override_child_frame = rospy.get_param('~override_child_frame', '')  # Empty = use camera_link

        # Covariance values for sensor fusion with IMU in Cartographer
        # Position: ORB-SLAM3 is quite accurate visually (cm-level in good conditions)
        self.position_covariance = rospy.get_param('~position_covariance', 0.005)  # 7cm std (5e-3 variance)

        # Orientation: Visual SLAM orientation is less reliable than IMU
        # Make it higher so Cartographer trusts IMU more for orientation
        self.orientation_covariance = rospy.get_param('~orientation_covariance', 0.05)  # ~13 deg std (5e-2 variance)

        # Velocity: Estimated from pose differences, less reliable
        self.velocity_covariance = rospy.get_param('~velocity_covariance', 0.5)  # Higher uncertainty

        # Scale factors for different conditions
        self.position_cov_scale_good = rospy.get_param('~position_cov_scale_good', 1.0)
        self.position_cov_scale_uncertain = rospy.get_param('~position_cov_scale_uncertain', 10.0)
        self.orientation_cov_scale_good = rospy.get_param('~orientation_cov_scale_good', 1.0)
        self.orientation_cov_scale_uncertain = rospy.get_param('~orientation_cov_scale_uncertain', 5.0)

        # Velocity estimation
        self.estimate_velocity = rospy.get_param('~estimate_velocity', True)
        self.velocity_alpha = rospy.get_param('~velocity_alpha', 0.8)  # Low-pass filter

        # State
        self.last_pose = None
        self.last_time = None
        self.last_published_time = None  # Track last published timestamp
        self.velocity = np.zeros(3)  # [vx, vy, vz]
        self.angular_velocity = np.zeros(3)  # [wx, wy, wz]
        self.pose_stable_count = 0  # Track consecutive stable poses for covariance adjustment

        # Timeout detection
        self.timeout_threshold = rospy.get_param('~timeout_threshold', 0.5)  # 500ms
        self.uncertain_covariance_scale = rospy.get_param('~uncertain_covariance_scale', 1000.0)  # 1000x
        self.was_timeout = False
        self.last_invalid_time = None  # Track when we first see invalid pose

        # Publisher & Subscriber
        self.odom_pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.pose_sub = rospy.Subscriber(self.input_topic, PoseStamped, self.pose_callback)

        rospy.loginfo("[PoseToOdom] Started")
        rospy.loginfo(f"  Input:  {self.input_topic} (PoseStamped)")
        rospy.loginfo(f"  Output: {self.output_topic} (Odometry)")
        if self.override_child_frame:
            rospy.loginfo(f"  Child frame override: {self.override_child_frame}")
        rospy.loginfo(f"  Velocity estimation: {self.estimate_velocity}")
        rospy.loginfo(f"  Timeout threshold: {self.timeout_threshold}s")
        rospy.loginfo(f"  Uncertain covariance scale: {self.uncertain_covariance_scale}x")

    def is_pose_invalid(self, pose):
        """
        Check if pose is invalid (ORB-SLAM3 not initialized/tracking lost)

        Invalid cases:
        1. All zeros: position=(0,0,0), orientation=(0,0,0,0)
        2. At origin with identity quaternion: position=(0,0,0), orientation=(0,0,0,1)

        Both cases indicate ORB-SLAM3 hasn't started tracking yet
        """
        position_at_origin = (abs(pose.position.x) < 1e-6 and
                              abs(pose.position.y) < 1e-6 and
                              abs(pose.position.z) < 1e-6)

        # Check if all components are zero
        all_zeros = (position_at_origin and
                     abs(pose.orientation.x) < 1e-6 and
                     abs(pose.orientation.y) < 1e-6 and
                     abs(pose.orientation.z) < 1e-6 and
                     abs(pose.orientation.w) < 1e-6)

        # Check if at origin with identity quaternion (w=1, x=y=z=0)
        identity_at_origin = (position_at_origin and
                              abs(pose.orientation.x) < 1e-6 and
                              abs(pose.orientation.y) < 1e-6 and
                              abs(pose.orientation.z) < 1e-6 and
                              abs(pose.orientation.w - 1.0) < 1e-6)

        return all_zeros or identity_at_origin

    def create_uncertain_odom(self, timestamp):
        """
        Create odometry with extremely high covariance for invalid pose

        Tells Cartographer: "Visual tracking lost, don't trust this!"
        Uses last tracked pose if available, otherwise zero pose.
        """
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'camera_link'

        if self.last_pose is not None:
            # Use last known valid pose but with HUGE uncertainty
            odom.pose.pose = self.last_pose
            pos_scale = self.uncertain_covariance_scale
            orient_scale = self.uncertain_covariance_scale
        else:
            # No previous valid pose - zero pose with max uncertainty
            odom.pose.pose.position.x = 0.0
            odom.pose.pose.position.y = 0.0
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.w = 1.0
            # Maximum uncertainty
            pos_scale = 1000000.0  # (1000m)^2 variance
            orient_scale = 100.0

        # Set very high pose covariance
        pose_cov = np.zeros(36)
        pose_cov[0] = self.position_covariance * pos_scale
        pose_cov[7] = self.position_covariance * pos_scale
        pose_cov[14] = self.position_covariance * pos_scale
        pose_cov[21] = self.orientation_covariance * orient_scale
        pose_cov[28] = self.orientation_covariance * orient_scale
        pose_cov[35] = self.orientation_covariance * orient_scale
        odom.pose.covariance = pose_cov.tolist()

        # Zero velocity with high uncertainty
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        twist_cov = np.zeros(36)
        twist_cov[0] = self.velocity_covariance * self.uncertain_covariance_scale
        twist_cov[7] = self.velocity_covariance * self.uncertain_covariance_scale
        twist_cov[14] = self.velocity_covariance * self.uncertain_covariance_scale
        twist_cov[21] = self.velocity_covariance * self.uncertain_covariance_scale * 2
        twist_cov[28] = self.velocity_covariance * self.uncertain_covariance_scale * 2
        twist_cov[35] = self.velocity_covariance * self.uncertain_covariance_scale * 2
        odom.twist.covariance = twist_cov.tolist()

        return odom

    def pose_callback(self, pose_msg):
        """
        Convert PoseStamped to Odometry

        Logic:
        1. Invalid pose (all 0) → Publish uncertain odom using last tracked pose
        2. Invalid pose timeout > threshold → Reset last tracked pose
        3. Valid pose → Compute and publish normal odom
        """
        pose = pose_msg.pose
        pose_time = pose_msg.header.stamp

        # Check if pose is invalid (all zeros - ORB-SLAM3 not tracking)
        if self.is_pose_invalid(pose):
            # Mark when we first saw invalid pose
            if self.last_invalid_time is None:
                self.last_invalid_time = rospy.Time.now()
                rospy.logwarn("[PoseToOdom] Invalid pose detected (all zeros). Publishing uncertain odom.")

            # Check if invalid pose has lasted too long
            invalid_duration = (rospy.Time.now() - self.last_invalid_time).to_sec()
            if invalid_duration > self.timeout_threshold:
                # Timeout exceeded - reset last tracked pose
                # Next valid poses will need 2 poses to restart odom calculation
                if self.last_pose is not None:
                    rospy.logwarn(f"[PoseToOdom] Invalid pose timeout ({invalid_duration:.2f}s). Resetting tracked pose.")
                    self.last_pose = None
                    self.last_time = None
                    self.velocity = np.zeros(3)
                    self.angular_velocity = np.zeros(3)
                    self.pose_stable_count = 0

            # Publish uncertain odom (using last tracked pose if available)
            uncertain_odom = self.create_uncertain_odom(pose_time)
            self.odom_pub.publish(uncertain_odom)
            return

        # Valid pose received
        # Reset invalid tracking
        if self.last_invalid_time is not None:
            rospy.loginfo("[PoseToOdom] Valid pose received. Resuming normal operation.")
            self.last_invalid_time = None

        odom = Odometry()

        # Use pose timestamp but ensure monotonic increasing
        pose_time = pose_msg.header.stamp

        # Check if this timestamp is older than last published (out of order)
        if self.last_published_time is not None and pose_time <= self.last_published_time:
            rospy.logwarn_throttle(1.0,
                f"[PoseToOdom] Out-of-order timestamp detected! "
                f"Last: {self.last_published_time.to_sec():.3f}, "
                f"Current: {pose_time.to_sec():.3f}. Skipping."
            )
            return

        odom.header.stamp = pose_time
        # This is camera odometry in map frame
        odom.header.frame_id = 'map'  # Camera pose is tracked in map frame

        # Child frame is camera_link (the camera body frame)
        # Cartographer will use TF to transform from camera_link to base_link
        if self.override_child_frame:
            odom.child_frame_id = self.override_child_frame
        else:
            odom.child_frame_id = 'camera_link'

        # Copy pose
        odom.pose.pose = pose_msg.pose

        # Check for timeout or first pose after recovery
        current_time = pose_time

        # If this is first valid pose ever or after invalid poses, skip it
        if self.last_pose is None or self.last_time is None:
            rospy.loginfo("[PoseToOdom] First valid pose received. Skipping to establish baseline.")
            self.last_pose = pose_msg.pose
            self.last_time = current_time
            self.last_published_time = current_time
            self.was_timeout = False
            return

        # Check for timeout (gap in pose updates)
        dt_since_last = (current_time - self.last_time).to_sec()
        if dt_since_last > self.timeout_threshold:
            # Timeout detected - tracking was lost or delayed
            rospy.logwarn(f"[PoseToOdom] Timeout detected! Gap: {dt_since_last:.2f}s. Skipping this pose.")

            # Reset velocity estimation state
            self.velocity = np.zeros(3)
            self.angular_velocity = np.zeros(3)
            self.pose_stable_count = 0

            # Skip this pose and use it as new baseline
            self.last_pose = pose_msg.pose
            self.last_time = current_time
            self.last_published_time = current_time
            self.was_timeout = True
            return
        else:
            # Normal operation - no timeout
            if self.was_timeout:
                rospy.loginfo("[PoseToOdom] Tracking stable again.")
                self.was_timeout = False

            # Increment stable count (capped at 10 for covariance scaling)
            self.pose_stable_count = min(self.pose_stable_count + 1, 10)

        # Estimate velocity if enabled
        if self.estimate_velocity and self.last_pose is not None and self.last_time is not None:
            dt = (current_time - self.last_time).to_sec()

            if dt > 0 and dt < self.timeout_threshold:  # Only if no timeout
                # Linear velocity
                dx = pose_msg.pose.position.x - self.last_pose.position.x
                dy = pose_msg.pose.position.y - self.last_pose.position.y
                dz = pose_msg.pose.position.z - self.last_pose.position.z

                vx = dx / dt
                vy = dy / dt
                vz = dz / dt

                # Low-pass filter
                self.velocity[0] = self.velocity_alpha * self.velocity[0] + (1 - self.velocity_alpha) * vx
                self.velocity[1] = self.velocity_alpha * self.velocity[1] + (1 - self.velocity_alpha) * vy
                self.velocity[2] = self.velocity_alpha * self.velocity[2] + (1 - self.velocity_alpha) * vz

                # Angular velocity (simplified - from yaw difference)
                # Extract yaw from quaternions
                _, _, yaw = euler_from_quaternion([
                    pose_msg.pose.orientation.x,
                    pose_msg.pose.orientation.y,
                    pose_msg.pose.orientation.z,
                    pose_msg.pose.orientation.w
                ])
                _, _, last_yaw = euler_from_quaternion([
                    self.last_pose.orientation.x,
                    self.last_pose.orientation.y,
                    self.last_pose.orientation.z,
                    self.last_pose.orientation.w
                ])

                # Angular velocity in z (yaw rate)
                dyaw = yaw - last_yaw
                # Handle angle wrap-around
                if dyaw > np.pi:
                    dyaw -= 2 * np.pi
                elif dyaw < -np.pi:
                    dyaw += 2 * np.pi

                wz = dyaw / dt
                self.angular_velocity[2] = self.velocity_alpha * self.angular_velocity[2] + (1 - self.velocity_alpha) * wz

        # Set velocity in odom
        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = self.velocity[2]
        odom.twist.twist.angular.x = self.angular_velocity[0]
        odom.twist.twist.angular.y = self.angular_velocity[1]
        odom.twist.twist.angular.z = self.angular_velocity[2]

        # Set covariance adaptively based on tracking stability
        # First few poses after recovery have higher uncertainty
        stability_factor = min(self.pose_stable_count / 5.0, 1.0)  # Ramps from 0 to 1 over 5 poses

        # Position covariance: Lower is better (Cartographer trusts more)
        # Scale up when just recovered, scale down when stable
        pos_scale = self.position_cov_scale_uncertain * (1.0 - stability_factor) + self.position_cov_scale_good * stability_factor

        # Orientation covariance: Keep higher than IMU so Cartographer trusts IMU more for orientation
        # Visual SLAM orientation drifts more than position
        orient_scale = self.orientation_cov_scale_uncertain * (1.0 - stability_factor) + self.orientation_cov_scale_good * stability_factor

        # Pose covariance (6x6 matrix, flattened to 36 elements)
        pose_cov = np.zeros(36)
        pose_cov[0] = self.position_covariance * pos_scale    # x
        pose_cov[7] = self.position_covariance * pos_scale    # y
        pose_cov[14] = self.position_covariance * pos_scale   # z
        pose_cov[21] = self.orientation_covariance * orient_scale  # roll - higher uncertainty, let IMU handle
        pose_cov[28] = self.orientation_covariance * orient_scale  # pitch - higher uncertainty, let IMU handle
        pose_cov[35] = self.orientation_covariance * orient_scale  # yaw - visual SLAM can help here but still defer to IMU
        odom.pose.covariance = pose_cov.tolist()

        # Twist covariance - velocity from visual odometry is less reliable than IMU
        twist_cov = np.zeros(36)
        twist_cov[0] = self.velocity_covariance   # vx
        twist_cov[7] = self.velocity_covariance   # vy
        twist_cov[14] = self.velocity_covariance  # vz
        twist_cov[21] = self.velocity_covariance * 2  # wx - IMU is better
        twist_cov[28] = self.velocity_covariance * 2  # wy - IMU is better
        twist_cov[35] = self.velocity_covariance * 2  # wz - IMU is better
        odom.twist.covariance = twist_cov.tolist()

        # Publish
        self.odom_pub.publish(odom)

        # Update state
        self.last_pose = pose_msg.pose
        self.last_time = current_time
        self.last_published_time = pose_time  # Track published timestamp for monotonic check

    def run(self):
        """Main loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        converter = PoseToOdom()
        converter.run()
    except rospy.ROSInterruptException:
        pass
