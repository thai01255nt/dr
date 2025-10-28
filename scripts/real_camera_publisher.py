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

        # Publishers with buffer size = 1 for minimal latency
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=1)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Camera parameters
        self.camera_id = rospy.get_param('~camera_id', 11)  # Default camera device ID (0 = /dev/video0)
        self.frame_id = rospy.get_param('~frame_id', 'camera')
        self.frame_rate = rospy.get_param('~frame_rate', 30)  # 20Hz
        self.image_width = 640  # Locked to 640x480 for OrangePi 5 Max optimization
        self.image_height = 480

        # Image encoding: 'bgr8' for color, 'mono8' for grayscale
        self.encoding = rospy.get_param('~encoding', 'mono8')  # Default to grayscale for performance

        # Use GStreamer hardware decoder for OrangePi 5 Max
        self.use_gstreamer = rospy.get_param('~use_gstreamer', True)

        # Convert to grayscale early (saves 66% data)
        self.convert_to_gray = rospy.get_param('~convert_to_gray', True)

        # Force resize disabled for locked resolution
        self.force_resize = False

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
        """Initialize camera device with hardware acceleration for OrangePi 5 Max"""
        try:
            if self.use_gstreamer:
                # GStreamer pipeline with hardware decoding for OrangePi 5 Max
                # Using v4l2src with hardware capabilities
                gst_pipeline = (
                    f"v4l2src device=/dev/video{self.camera_id} ! "
                    f"video/x-raw,width={self.image_width},height={self.image_height},framerate={self.frame_rate}/1 ! "
                    f"videoconvert ! "
                    f"appsink max-buffers=1 drop=true"
                )

                rospy.loginfo(f"Using GStreamer hardware pipeline: {gst_pipeline}")
                self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            else:
                # Fallback to V4L2 backend
                self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)

            if not self.cap.isOpened():
                rospy.logerr(f"Failed to open camera device {self.camera_id}")
                return False

            if not self.use_gstreamer:
                # Set camera resolution and parameters for V4L2 backend
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
                self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)

                # Set buffer size to 1 for minimal latency
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                # Disable auto focus and auto exposure for consistent performance
                self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

                # Set FOURCC format to MJPEG for better performance
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))

            # Verify settings
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

            rospy.loginfo(f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps}Hz")
            rospy.loginfo(f"GStreamer: {self.use_gstreamer}, Buffer size: 1, Grayscale: {self.convert_to_gray}")

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
        """Capture image from camera and publish to ROS topic - optimized for minimal processing"""
        if self.cap is None or not self.cap.isOpened():
            rospy.logwarn_throttle(5.0, "Camera not available")
            return False

        try:
            # Capture frame from camera (already at 640x480 from hardware)
            ret, frame = self.cap.read()

            if not ret or frame is None:
                rospy.logwarn_throttle(5.0, "Failed to capture frame from camera")
                return False

            # Early grayscale conversion (saves 66% data processing)
            if self.convert_to_gray and self.encoding == 'mono8':
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Get current ROS timestamp
            timestamp = rospy.Time.now()

            # Convert to ROS Image message (minimal overhead)
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
        """Main loop - optimized for OrangePi 5 Max"""
        rospy.loginfo("="*50)
        rospy.loginfo("Real Camera Publisher node started (OrangePi 5 Max Optimized)")
        rospy.loginfo(f"Camera device: /dev/video{self.camera_id}")
        rospy.loginfo(f"Publishing to:")
        rospy.loginfo(f"  - /camera/image_raw")
        rospy.loginfo(f"  - /camera/camera_info")
        rospy.loginfo(f"Frame rate: {self.frame_rate}Hz")
        rospy.loginfo(f"Resolution: {self.image_width}x{self.image_height} (locked)")
        rospy.loginfo(f"Encoding: {self.encoding}")
        rospy.loginfo(f"GStreamer HW accel: {self.use_gstreamer}")
        rospy.loginfo(f"Early grayscale: {self.convert_to_gray}")
        rospy.loginfo(f"Buffer size: 1 (minimal latency)")
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
