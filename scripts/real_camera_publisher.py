#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import signal
import sys

class RealCameraPublisher:
    def __init__(self):
        rospy.init_node('real_camera_publisher', anonymous=False)

        # Publishers
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=1)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Camera parameters
        self.camera_id = rospy.get_param('~camera_id', 0)  # Default camera device ID (0 = /dev/video0)
        self.frame_id = rospy.get_param('~frame_id', 'camera')
        self.frame_rate = rospy.get_param('~frame_rate', 20)  # 20Hz
        self.image_width = rospy.get_param('~image_width', 640)  # VGA width
        self.image_height = rospy.get_param('~image_height', 480)  # VGA height

        # Image encoding: 'bgr8' for color, 'mono8' for grayscale
        self.encoding = rospy.get_param('~encoding', 'bgr8')  # Default to color

        # Force resize if camera doesn't support requested resolution
        self.force_resize = rospy.get_param('~force_resize', False)

        # Camera intrinsics - adjust based on your camera calibration
        self.fx = rospy.get_param('~fx', 322.5)  # focal length x
        self.fy = rospy.get_param('~fy', 325)    # focal length y
        self.cx = rospy.get_param('~cx', 320)    # principal point x
        self.cy = rospy.get_param('~cy', 240)    # principal point y

        # Distortion coefficients (k1, k2, p1, p2, k3) - radtan model
        self.k1 = rospy.get_param('~k1', 0.0)
        self.k2 = rospy.get_param('~k2', 0.0)
        self.p1 = rospy.get_param('~p1', 0.0)
        self.p2 = rospy.get_param('~p2', 0.0)
        self.k3 = rospy.get_param('~k3', 0.0)

        # Rate
        self.rate = rospy.Rate(self.frame_rate)

        # Shutdown flag
        self.shutdown_requested = False

        # OpenCV VideoCapture
        self.cap = None
        self.init_camera()

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

        # Camera info message (static, publish once per image)
        self.camera_info_msg = self.create_camera_info_msg()

    def init_camera(self):
        """Initialize camera device"""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)

            if not self.cap.isOpened():
                rospy.logerr(f"Failed to open camera device {self.camera_id}")
                return False

            # Set camera resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)

            # Set FPS if possible
            self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)

            # Verify settings
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

            rospy.loginfo(f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps}Hz")

            if actual_width != self.image_width or actual_height != self.image_height:
                if self.force_resize:
                    rospy.logwarn(f"Requested resolution {self.image_width}x{self.image_height}, " +
                                f"got {actual_width}x{actual_height}. Will resize frames.")
                else:
                    rospy.logwarn(f"Requested resolution {self.image_width}x{self.image_height}, " +
                                f"got {actual_width}x{actual_height}. Using camera native resolution.")
                    # Update to actual camera resolution
                    self.image_width = actual_width
                    self.image_height = actual_height
                    # Update camera info with actual dimensions
                    self.camera_info_msg = self.create_camera_info_msg()

            return True

        except Exception as e:
            rospy.logerr(f"Failed to initialize camera: {e}")
            return False

    def shutdown_hook(self):
        """Cleanup on shutdown"""
        rospy.loginfo("Shutting down Real Camera Publisher node...")
        self.shutdown_requested = True

        if self.cap is not None:
            try:
                self.cap.release()
                rospy.loginfo("Camera device released")
            except Exception as e:
                rospy.logwarn(f"Error during camera release: {e}")

    def create_camera_info_msg(self):
        """Create CameraInfo message"""
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

    def capture_and_publish(self):
        """Capture image from camera and publish to ROS topic"""
        if self.cap is None or not self.cap.isOpened():
            rospy.logwarn_throttle(5.0, "Camera not available")
            return False

        try:
            # Capture frame from camera
            ret, frame = self.cap.read()

            if not ret or frame is None:
                rospy.logwarn_throttle(5.0, "Failed to capture frame from camera")
                return False

            # Convert to grayscale if needed
            if self.encoding == 'mono8':
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Resize if force_resize is enabled and dimensions don't match
            if self.force_resize and (frame.shape[0] != self.image_height or frame.shape[1] != self.image_width):
                frame = cv2.resize(frame, (self.image_width, self.image_height))
                rospy.logwarn_throttle(10.0, f"Resizing frame from {frame.shape[1]}x{frame.shape[0]} to {self.image_width}x{self.image_height}")

            # Get current ROS timestamp
            timestamp = rospy.Time.now()

            # Convert to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding=self.encoding)
            image_msg.header.stamp = timestamp
            image_msg.header.frame_id = self.frame_id

            # Update camera info timestamp
            self.camera_info_msg.header.stamp = timestamp

            # Publish both image and camera info
            self.image_pub.publish(image_msg)
            self.camera_info_pub.publish(self.camera_info_msg)

            return True

        except Exception as e:
            rospy.logerr(f"Error capturing/publishing image: {e}")
            return False

    def run(self):
        """Main loop"""
        rospy.loginfo("="*50)
        rospy.loginfo("Real Camera Publisher node started")
        rospy.loginfo(f"Camera device: /dev/video{self.camera_id}")
        rospy.loginfo(f"Publishing to:")
        rospy.loginfo(f"  - /camera/image_raw")
        rospy.loginfo(f"  - /camera/camera_info")
        rospy.loginfo(f"Frame rate: {self.frame_rate}Hz")
        rospy.loginfo(f"Resolution: {self.image_width}x{self.image_height}")
        rospy.loginfo(f"Encoding: {self.encoding}")
        rospy.loginfo(f"Force resize: {self.force_resize}")
        rospy.loginfo(f"Using ROS Time.now() for timestamps")
        rospy.loginfo("="*50)

        while not rospy.is_shutdown() and not self.shutdown_requested:
            try:
                # Capture and publish image
                self.capture_and_publish()

                # Sleep to maintain frame rate
                self.rate.sleep()

            except KeyboardInterrupt:
                rospy.loginfo("KeyboardInterrupt received, shutting down...")
                break
            except Exception as e:
                if not rospy.is_shutdown():
                    rospy.logerr(f"Error in main loop: {e}")

        rospy.loginfo("Real Camera Publisher node stopped.")

def signal_handler(sig, frame):
    """Handle Ctrl+C signal"""
    rospy.loginfo("SIGINT received, shutting down gracefully...")
    rospy.signal_shutdown("SIGINT received")
    sys.exit(0)

if __name__ == '__main__':
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    try:
        node = RealCameraPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received, exiting...")
    finally:
        rospy.loginfo("Node terminated.")
