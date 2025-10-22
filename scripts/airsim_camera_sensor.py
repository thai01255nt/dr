#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cosysairsim as airsim
import numpy as np
import cv2
import signal
import sys

class AirSimCameraNode:
    def __init__(self):
        rospy.init_node('airsim_camera_node', anonymous=False)

        # Publishers
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=1)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # AirSim parameters
        self.vehicle_name = rospy.get_param('~vehicle_name', 'Copter')
        self.camera_name = rospy.get_param('~camera_name', '0')  # Default camera

        # Camera parameters - Optimized for VinsMono on OrangePi 5 Max
        self.frame_id = rospy.get_param('~frame_id', 'camera')
        self.frame_rate = rospy.get_param('~frame_rate', 20)  # 20Hz - balance between smoothness and CPU
        self.image_width = rospy.get_param('~image_width', 640)  # VGA width
        self.image_height = rospy.get_param('~image_height', 480)  # VGA height

        # Camera intrinsics - typical values for 640x480, adjust based on your camera calibration
        # For VinsMono, you should calibrate your camera and update these values
        self.fx = rospy.get_param('~fx', 322.5)  # focal length x
        self.fy = rospy.get_param('~fy', 325)  # focal length y
        self.cx = rospy.get_param('~cx', 325)  # principal point x
        self.cy = rospy.get_param('~cy', 238)  # principal point y

        # Distortion coefficients (k1, k2, p1, p2, k3) - radtan model for VinsMono
        self.k1 = rospy.get_param('~k1', -0.151)
        self.k2 = rospy.get_param('~k2', 0.0798)
        self.p1 = rospy.get_param('~p1', 0.001003)
        self.p2 = rospy.get_param('~p2', 0.00102)
        self.k3 = rospy.get_param('~k3', -0.02)

        # Rate
        self.rate = rospy.Rate(self.frame_rate)

        # Shutdown flag
        self.shutdown_requested = False

        # Connect to AirSim
        self.client = None
        self.connect_to_airsim()

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

        # Camera info message (static, publish once per image)
        self.camera_info_msg = self.create_camera_info_msg()

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
        rospy.loginfo("Shutting down AirSim Camera node...")
        self.shutdown_requested = True
        if self.client is not None:
            try:
                self.client = None
            except Exception as e:
                rospy.logwarn(f"Error during shutdown: {e}")

    def create_camera_info_msg(self):
        """Create CameraInfo message for VinsMono"""
        camera_info = CameraInfo()

        camera_info.header.frame_id = self.frame_id

        # Image dimensions
        camera_info.height = self.image_height
        camera_info.width = self.image_width

        # Distortion model
        camera_info.distortion_model = "plumb_bob"  # radtan model

        # Distortion coefficients D = [k1, k2, p1, p2, k3]
        camera_info.D = [self.k1, self.k2, self.p1, self.p2, self.k3]

        # Intrinsic camera matrix K
        # [fx  0  cx]
        # [ 0 fy  cy]
        # [ 0  0   1]
        camera_info.K = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0
        ]

        # Rectification matrix R (identity for monocular)
        camera_info.R = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix P
        # For monocular: P = K * [I | 0]
        # [fx  0  cx  0]
        # [ 0 fy  cy  0]
        # [ 0  0   1  0]
        camera_info.P = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # Binning and ROI
        camera_info.binning_x = 0
        camera_info.binning_y = 0
        camera_info.roi.x_offset = 0
        camera_info.roi.y_offset = 0
        camera_info.roi.height = 0
        camera_info.roi.width = 0
        camera_info.roi.do_rectify = False

        return camera_info

    def get_camera_image(self):
        """Get camera image from AirSim"""
        if self.client is None:
            return None

        try:
            # Request grayscale image (VinsMono uses grayscale)
            # ImageType.Scene = 0 (RGB), ImageType.DepthPlanar = 1, ImageType.Segmentation = 5
            # For VinsMono, we want grayscale, so get RGB first then convert
            # responses = self.client.simGetImages([
            #     airsim.ImageRequest(
            #         self.camera_name,
            #         airsim.ImageType.Scene,
            #         False,  # uncompressed
            #         False   # no PNG compression
            #     )
            # ], vehicle_name=self.vehicle_name)
            responses = self.client.simGetImages([
                airsim.ImageRequest(
                    self.camera_name,
                    airsim.ImageType.Scene,
                    False,  # uncompressed
                    False   # no PNG compression
                )
            ], vehicle_name=self.vehicle_name)
            if len(responses) == 0 or responses[0] is None:
                return None

            response = responses[0]

            # Convert to numpy array
            if response.image_type == airsim.ImageType.Scene:
                # RGB image
                img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)

                # Reshape to RGB image
                img_rgb = img1d.reshape(response.height, response.width, 3)

                # Convert BGR to RGB (AirSim gives BGR)
                img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)

                # Convert to grayscale for VinsMono (saves bandwidth and processing)
                img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)

                # Resize if needed (for VinsMono optimization)
                if img_gray.shape[0] != self.image_height or img_gray.shape[1] != self.image_width:
                    img_gray = cv2.resize(img_gray, (self.image_width, self.image_height))

                return img_gray

            return None

        except Exception as e:
            rospy.logerr(f"Error getting camera image: {e}")
            return None

    def publish_image(self, img_gray):
        """Publish grayscale image and camera info"""
        try:
            # Create timestamp
            timestamp = rospy.Time.now()

            # Convert grayscale image to ROS Image message
            # VinsMono expects mono8 encoding
            image_msg = self.bridge.cv2_to_imgmsg(img_gray, encoding="mono8")
            image_msg.header.stamp = timestamp
            image_msg.header.frame_id = self.frame_id

            # Update camera info timestamp
            self.camera_info_msg.header.stamp = timestamp

            # Publish both image and camera info
            self.image_pub.publish(image_msg)
            self.camera_info_pub.publish(self.camera_info_msg)

        except Exception as e:
            rospy.logerr(f"Error publishing image: {e}")

    def run(self):
        """Main loop"""
        rospy.loginfo(f"AirSim Camera node started")
        rospy.loginfo(f"Publishing to /camera/image_raw at {self.frame_rate}Hz")
        rospy.loginfo(f"Resolution: {self.image_width}x{self.image_height} (grayscale for VinsMono)")
        rospy.loginfo(f"Optimized for OrangePi 5 Max")

        while not rospy.is_shutdown() and not self.shutdown_requested:
            try:
                # Get camera image from AirSim
                img = self.get_camera_image()

                if img is not None:
                    # Publish image and camera info
                    self.publish_image(img)
                else:
                    rospy.logwarn_throttle(5.0, "No image received from AirSim")

                self.rate.sleep()

            except KeyboardInterrupt:
                rospy.loginfo("KeyboardInterrupt received, shutting down...")
                break
            except Exception as e:
                if not rospy.is_shutdown():
                    rospy.logerr(f"Error in main loop: {e}")

        rospy.loginfo("AirSim Camera node stopped.")

def signal_handler(sig, frame):
    """Handle Ctrl+C signal"""
    rospy.loginfo("SIGINT received, shutting down gracefully...")
    rospy.signal_shutdown("SIGINT received")
    sys.exit(0)

if __name__ == '__main__':
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    try:
        node = AirSimCameraNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received, exiting...")
    finally:
        rospy.loginfo("Node terminated.")
