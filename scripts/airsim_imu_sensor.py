#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import cosysairsim as airsim
import numpy as np
import signal
import sys

class AirSimImuNode:
    def __init__(self):
        rospy.init_node('airsim_imu_node', anonymous=False)

        # Publisher
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=1)

        # AirSim parameters
        self.vehicle_name = rospy.get_param('~vehicle_name', 'Copter')
        self.imu_name = rospy.get_param('~imu_name', 'Imu')  # Default IMU sensor name

        # IMU parameters
        self.frame_id = rospy.get_param('~frame_id', 'base_link')
        self.imu_rate = rospy.get_param('~imu_rate', 4)  # 100Hz - typical for IMU

        # Rate
        self.rate = rospy.Rate(self.imu_rate)

        # Shutdown flag
        self.shutdown_requested = False

        # Connect to AirSim
        self.client = None
        self.connect_to_airsim()

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

        # IMU message template
        self.imu_msg = self.create_imu_msg_template()

    def connect_to_airsim(self):
        """Connect to AirSim"""
        try:
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            rospy.loginfo("Connected to AirSim")
        except Exception as e:
            rospy.logerr(f"Failed to connect to AirSim: {e}")
            self.client = None

    def shutdown_hook(self):
        """Cleanup on shutdown"""
        rospy.loginfo("Shutting down AirSim IMU node...")
        self.shutdown_requested = True
        if self.client is not None:
            try:
                self.client = None
            except Exception as e:
                rospy.logwarn(f"Error during shutdown: {e}")

    def create_imu_msg_template(self):
        """Create IMU message template"""
        imu_msg = Imu()
        imu_msg.header.frame_id = self.frame_id

        # Orientation covariance (set to -1 if orientation not provided)
        imu_msg.orientation_covariance = [-1, 0, 0,
                                        0, 0, 0,
                                        0, 0, 0]

        # Angular velocity covariance (rad/s)^2
        # From AirSim: AngularRandomWalk ≈ 0.005 → σ = 0.005 → cov = 0.005^2 = 2.5e-5
        imu_msg.angular_velocity_covariance = [2.5e-05, 0, 0,
                                            0, 2.5e-05, 0,
                                            0, 0, 2.5e-05]

        # Linear acceleration covariance (m/s²)^2
        # From AirSim: VelocityRandomWalk ≈ 0.02 → σ = 0.02 → cov = 0.02^2 = 4.0e-4
        imu_msg.linear_acceleration_covariance = [4.0e-04, 0, 0,
                                                0, 4.0e-04, 0,
                                                0, 0, 4.0e-04]

        return imu_msg

    def get_imu_data(self):
        """Get IMU data from AirSim"""
        if self.client is None:
            return None

        try:
            # Get IMU data from AirSim
            imu_data = self.client.getImuData(imu_name=self.imu_name, vehicle_name=self.vehicle_name)
            
            if imu_data is None:
                return None

            return imu_data

        except Exception as e:
            rospy.logerr(f"Error getting IMU data: {e}")
            return None

    def publish_imu(self, imu_data):
        """Publish IMU data"""
        try:
            # Create timestamp (same logic as camera node)
            # timestamp_ns = imu_data.time_stamp
            # secs = timestamp_ns // 1_000_000_000
            # nsecs = timestamp_ns % 1_000_000_000
            # timestamp = rospy.Time(secs, nsecs)
            timestamp = rospy.Time.now()
            # Update header
            self.imu_msg.header.stamp = timestamp

            # Orientation (quaternion)
            # AirSim uses NED coordinate system, convert to ENU (ROS standard)
            # NED to ENU quaternion: qw_enu = qw_ned, qx_enu = qy_ned, qy_enu = qx_ned, qz_enu = -qz_ned
            self.imu_msg.orientation.w = imu_data.orientation.w_val
            self.imu_msg.orientation.x = imu_data.orientation.y_val
            self.imu_msg.orientation.y = imu_data.orientation.x_val
            self.imu_msg.orientation.z = -imu_data.orientation.z_val

            # Angular velocity (rad/s)
            # AirSim uses NED coordinate system, convert to ENU (ROS standard)
            # NED to ENU: x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
            self.imu_msg.angular_velocity.x = imu_data.angular_velocity.y_val
            self.imu_msg.angular_velocity.y = imu_data.angular_velocity.x_val
            self.imu_msg.angular_velocity.z = -imu_data.angular_velocity.z_val

            # Linear acceleration (m/s^2)
            # AirSim uses NED coordinate system, convert to ENU (ROS standard)
            # NED to ENU: x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
            self.imu_msg.linear_acceleration.x = imu_data.linear_acceleration.y_val
            self.imu_msg.linear_acceleration.y = imu_data.linear_acceleration.x_val
            self.imu_msg.linear_acceleration.z = -imu_data.linear_acceleration.z_val

            # Publish IMU message
            self.imu_pub.publish(self.imu_msg)

        except Exception as e:
            rospy.logerr(f"Error publishing IMU data: {e}")

    def run(self):
        """Main loop"""
        rospy.loginfo(f"AirSim IMU node started")
        rospy.loginfo(f"Publishing to /imu/data_raw at {self.imu_rate}Hz")
        rospy.loginfo(f"Vehicle: {self.vehicle_name}, IMU: {self.imu_name}")

        while not rospy.is_shutdown() and not self.shutdown_requested:
            try:
                # Get IMU data from AirSim
                imu_data = self.get_imu_data()

                if imu_data is not None:
                    # Publish IMU data
                    self.publish_imu(imu_data)
                else:
                    rospy.logwarn_throttle(5.0, "No IMU data received from AirSim")

                self.rate.sleep()

            except KeyboardInterrupt:
                rospy.loginfo("KeyboardInterrupt received, shutting down...")
                break
            except Exception as e:
                if not rospy.is_shutdown():
                    rospy.logerr(f"Error in main loop: {e}")

        rospy.loginfo("AirSim IMU node stopped.")

def signal_handler(sig, frame):
    """Handle Ctrl+C signal"""
    rospy.loginfo("SIGINT received, shutting down gracefully...")
    rospy.signal_shutdown("SIGINT received")
    sys.exit(0)

if __name__ == '__main__':
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    try:
        node = AirSimImuNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received, exiting...")
    finally:
        rospy.loginfo("Node terminated.")