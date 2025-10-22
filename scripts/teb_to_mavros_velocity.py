#!/usr/bin/env python3
"""
Convert TEB velocity commands from body frame to local frame for Ardupilot UAV
Subscribes to:
- /mavros/local_position/pose (PoseStamped) - current pose from Ardupilot
- /teb_local_planner/cmd_vel_unstamped (Twist) - velocity commands in body frame from TEB

Publishes to:
- /mavros/setpoint_velocity/cmd_vel_unstamped (Twist) - velocity commands in local frame for Ardupilot
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import quaternion_matrix


class TebToMavrosVelocityConverter:
    def __init__(self):
        rospy.init_node('teb_to_mavros_velocity_converter', anonymous=True)

        # Current orientation quaternion from Ardupilot
        self.current_quaternion = [0.0, 0.0, 0.0, 1.0]  # [x, y, z, w]
        self.rotation_matrix = np.eye(3)
        self.pose_received = False

        # Altitude hold parameters
        self.altitude_hold = rospy.get_param('~altitude_hold', True)  # Enable altitude hold by default
        rospy.loginfo(f"Altitude hold: {'ENABLED' if self.altitude_hold else 'DISABLED'}")

        # Publishers
        self.cmd_vel_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            Twist,
            queue_size=10
        )

        # Subscribers
        self.pose_sub = rospy.Subscriber(
            '/mavros/local_position/pose',
            PoseStamped,
            self.pose_callback,
            queue_size=10
        )

        self.teb_cmd_sub = rospy.Subscriber(
            '/teb_local_planner/cmd_vel_unstamped',
            Twist,
            self.teb_cmd_callback,
            queue_size=10
        )

        rospy.loginfo("TEB to Mavros Velocity Converter initialized")
        rospy.loginfo("Waiting for pose data from /mavros/local_position/pose...")

    def pose_callback(self, msg):
        """
        Extract orientation quaternion and compute rotation matrix
        """
        # Extract quaternion
        orientation_q = msg.pose.orientation
        self.current_quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # Convert quaternion to 4x4 transformation matrix
        transformation_matrix = quaternion_matrix(self.current_quaternion)

        # Extract 3x3 rotation matrix (body to local frame)
        self.rotation_matrix = transformation_matrix[:3, :3]

        if not self.pose_received:
            rospy.loginfo("Pose data received. Ready to convert velocity commands.")
            self.pose_received = True

    def teb_cmd_callback(self, msg):
        """
        Convert velocity command from body frame to local frame using full 3D rotation
        Body frame (TEB): x-forward, y-left, z-up (relative to UAV body)
        Local frame (Ardupilot): typically ENU (East-North-Up) or NED (North-East-Down)

        Uses full 3x3 rotation matrix from quaternion to handle roll, pitch, yaw
        """
        if not self.pose_received:
            rospy.logwarn_throttle(5.0, "Pose not yet received. Skipping velocity conversion.")
            return

        # Extract body frame velocities from TEB
        vel_body = np.array([msg.linear.x, msg.linear.y, msg.linear.z])

        # Apply full 3D rotation matrix: v_local = R * v_body
        vel_local = self.rotation_matrix.dot(vel_body)

        # Create output message in local frame
        cmd_vel_local = Twist()
        cmd_vel_local.linear.x = vel_local[0]
        cmd_vel_local.linear.y = vel_local[1]

        # Altitude hold: force vz = 0 to maintain current altitude
        if self.altitude_hold:
            cmd_vel_local.linear.z = 0.0
        else:
            cmd_vel_local.linear.z = vel_local[2]

        # Angular velocity needs to be rotated as well
        omega_body = np.array([msg.angular.x, msg.angular.y, msg.angular.z])
        omega_local = self.rotation_matrix.dot(omega_body)

        cmd_vel_local.angular.x = omega_local[0]
        cmd_vel_local.angular.y = omega_local[1]
        cmd_vel_local.angular.z = omega_local[2]

        # Publish converted velocity command
        self.cmd_vel_pub.publish(cmd_vel_local)

        # Debug output (throttled to avoid spam)
        rospy.logdebug(
            f"Body frame: vx={vel_body[0]:.3f}, vy={vel_body[1]:.3f}, vz={vel_body[2]:.3f} | "
            f"Local frame: vx={vel_local[0]:.3f}, vy={vel_local[1]:.3f}, vz={cmd_vel_local.linear.z:.3f} "
            f"{'(altitude hold)' if self.altitude_hold else ''}"
        )

    def run(self):
        """
        Keep the node running
        """
        rospy.spin()


if __name__ == '__main__':
    try:
        converter = TebToMavrosVelocityConverter()
        converter.run()
    except rospy.ROSInterruptException:
        pass
