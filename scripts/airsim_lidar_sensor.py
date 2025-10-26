#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import airsim
import math
import numpy as np
import signal
import sys

class Ld19AirSimNode:
    def __init__(self):
        rospy.init_node('ld19_airsim_node', anonymous=False)

        # Publisher - giống LD19 driver
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

        # AirSim parameters
        self.vehicle_name = rospy.get_param('~vehicle_name', 'Copter')
        self.lidar_name = rospy.get_param('~lidar_name', 'Ld19')

        # LD19 parameters - giống LD19 ROS driver
        self.frame_id = rospy.get_param('~frame_id', 'base_laser')
        self.scan_frequency = rospy.get_param('~scan_frequency', 10)  # Hz

        # LD19 specs
        self.range_min = 0.02   # 2cm
        self.range_max = 12.0   # 12m

        # Rate
        self.rate = rospy.Rate(self.scan_frequency)

        # Shutdown flag
        self.shutdown_requested = False

        # Connect to AirSim
        self.client = None
        self.connect_to_airsim()

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

    def connect_to_airsim(self):
        """Kết nối tới AirSim"""
        try:
            # Dùng MultirotorClient cho drone, CarClient cho car
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            rospy.loginfo("Connected to AirSim")
        except Exception as e:
            rospy.logerr(f"Failed to connect to AirSim: {e}")
            self.client = None

    def shutdown_hook(self):
        """Cleanup khi shutdown"""
        rospy.loginfo("Shutting down LD19 AirSim node...")
        self.shutdown_requested = True
        if self.client is not None:
            try:
                # Disconnect from AirSim if needed
                self.client = None
            except Exception as e:
                rospy.logwarn(f"Error during shutdown: {e}")

    def get_lidar_data(self):
        """Lấy dữ liệu lidar từ AirSim"""
        if self.client is None:
            return None

        try:
            lidar_data = self.client.getLidarData(
                lidar_name=self.lidar_name,
                vehicle_name=self.vehicle_name
            )

            if len(lidar_data.point_cloud) < 3:
                return None

            return lidar_data

        except Exception as e:
            rospy.logerr(f"Error getting lidar data: {e}")
            return None

    def convert_ned_to_ld19(self, points):
        """
        Chuyển từ AirSim NED sang LD19 coordinate
        AirSim NED: X=forward, Y=right, Z=down
        LD19 ROS: X=forward, Y=left, Z=up

        Transform:
        X_ld19 = X_ned
        Y_ld19 = -Y_ned
        Z_ld19 = -Z_ned
        """
        points_ld19 = np.copy(points)
        points_ld19[:, 1] = -points[:, 1]  # Y flip
        points_ld19[:, 2] = -points[:, 2]  # Z flip
        return points_ld19

    def pointcloud_to_laserscan(self, lidar_data):
        """
        Chuyển point cloud sang LaserScan
        LD19 quét clockwise, góc tăng dần từ 0 đến 2π
        """
        # Parse point cloud
        points = np.array(lidar_data.point_cloud, dtype=np.float32)
        points = points.reshape((-1, 3))

        # Convert NED to LD19 coordinate
        points = self.convert_ned_to_ld19(points)

        # LD19 có ~4500 points/sec, với 10Hz scan = ~450 points/scan
        # Nhưng để tương thích, dùng 360 beams (1 degree resolution)
        num_beams = 360
        angle_min = 0.0  # LD19 bắt đầu từ 0
        angle_max = 2.0 * math.pi  # đến 2π
        angle_increment = (angle_max - angle_min) / num_beams

        # Initialize ranges với inf
        ranges = [float('inf')] * num_beams

        # Convert points to ranges
        for point in points:
            x, y, z = point

            # Distance 2D (bỏ qua z cho 2D scan)
            distance = math.sqrt(x*x + y*y)

            # Bỏ qua points ngoài range
            if distance < self.range_min or distance > self.range_max:
                continue

            # Angle - LD19 dùng atan2(y,x) và quét clockwise
            # Trong ROS REP-103: angle tăng ngược chiều kim đồng hồ
            # Nhưng LD19 quét theo chiều kim đồng hồ, nên cần adjust
            angle = math.atan2(y, x)

            # Normalize angle to [0, 2π]
            if angle < 0:
                angle += 2.0 * math.pi

            # Find corresponding beam index
            index = int(angle / angle_increment)

            if 0 <= index < num_beams:
                # Keep minimum distance for each beam
                if distance < ranges[index]:
                    ranges[index] = distance

        return ranges, angle_min, angle_max, angle_increment

    def create_laserscan_msg(self, ranges, angle_min, angle_max, angle_increment, timestamp):
        """Tạo LaserScan message - format giống LD19 driver"""
        scan = LaserScan()
        # secs = timestamp // 1_000_000_000
        # nsecs = timestamp % 1_000_000_000
        # timestamp = rospy.Time(secs, nsecs)
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = self.frame_id

        # LD19 specifications
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0  # LD19 không report time increment
        scan.scan_time = 1.0 / self.scan_frequency

        scan.range_min = self.range_min
        scan.range_max = self.range_max

        scan.ranges = ranges
        scan.intensities = []  # LD19 không có intensity data

        return scan

    def run(self):
        """Main loop - giống LD19 driver"""
        rospy.loginfo(f"LD19 AirSim node started, publishing to /scan at {self.scan_frequency}Hz")

        while not rospy.is_shutdown() and not self.shutdown_requested:
            try:
                # Get lidar data from AirSim
                lidar_data = self.get_lidar_data()

                if lidar_data is not None:
                    # Convert to LaserScan
                    ranges, angle_min, angle_max, angle_increment = \
                        self.pointcloud_to_laserscan(lidar_data)

                    # Create and publish message
                    scan_msg = self.create_laserscan_msg(
                        ranges, angle_min, angle_max, angle_increment, lidar_data.time_stamp
                    )
                    self.scan_pub.publish(scan_msg)

                self.rate.sleep()

            except KeyboardInterrupt:
                rospy.loginfo("KeyboardInterrupt received, shutting down...")
                break
            except Exception as e:
                if not rospy.is_shutdown():
                    rospy.logerr(f"Error in main loop: {e}")

        rospy.loginfo("LD19 AirSim node stopped.")

def signal_handler(sig, frame):
    """Handle Ctrl+C signal"""
    rospy.loginfo("SIGINT received, shutting down gracefully...")
    rospy.signal_shutdown("SIGINT received")
    sys.exit(0)

if __name__ == '__main__':
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    try:
        node = Ld19AirSimNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received, exiting...")
    finally:
        rospy.loginfo("Node terminated.")
