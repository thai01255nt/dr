#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class RealCameraPublisher:
    def __init__(self):
        rospy.init_node('real_camera_publisher', anonymous=False)

        # Parameters
        self.camera_id = rospy.get_param('~camera_id', 11)
        self.frame_id = rospy.get_param('~frame_id', 'camera')
        self.use_gray = rospy.get_param('~use_gray', False)  # True = grayscale, False = BGR
        self.use_gstreamer = rospy.get_param('~use_gstreamer', True)  # GStreamer hardware acceleration

        # Publishers - queue_size=1 for minimum latency
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=1)

        self.bridge = CvBridge()

        # Target resolution
        self.width = 640
        self.height = 480

        # Initialize camera
        self.cap = None
        self.init_camera()

        # Camera info
        self.camera_info = self.create_camera_info()

        rospy.on_shutdown(self.shutdown)

    def init_camera(self):
        """Initialize OV13855 camera on OrangePi 5 Max"""
        if self.use_gstreamer:
            # GStreamer with hardware acceleration
            # RK3588 can do hardware scaling and format conversion
            gst_str = (
                f"v4l2src device=/dev/video{self.camera_id} io-mode=dmabuf ! "
                f"video/x-raw,format=NV12 ! "
                f"videoscale ! video/x-raw,width={self.width},height={self.height} ! "
                f"videoconvert ! video/x-raw,format={'GRAY8' if self.use_gray else 'BGR'} ! "
                f"appsink max-buffers=1 drop=true sync=false"
            )

            rospy.loginfo(f"GStreamer pipeline: {gst_str}")
            self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        else:
            # V4L2 direct access
            self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)

            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not self.cap.isOpened():
            rospy.logerr(f"Cannot open /dev/video{self.camera_id}")
            if self.use_gstreamer:
                rospy.logerr("Try: _use_gstreamer:=false")
            return False

        # Get actual resolution
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        rospy.loginfo("="*50)
        rospy.loginfo(f"Camera: /dev/video{self.camera_id} (OV13855)")
        rospy.loginfo(f"Backend: {'GStreamer HW' if self.use_gstreamer else 'V4L2'}")
        rospy.loginfo(f"Resolution: {actual_w}x{actual_h}")
        rospy.loginfo(f"Target: {self.width}x{self.height}")
        rospy.loginfo(f"Mode: {'GRAYSCALE' if self.use_gray else 'BGR COLOR'}")
        rospy.loginfo(f"Buffer: 1 (minimal latency)")
        rospy.loginfo("="*50)

        return True

    def create_camera_info(self):
        """Create camera info message"""
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.height = self.height
        info.width = self.width
        info.distortion_model = "plumb_bob"

        # Calibration for 640x480 (adjust if you have calibration file)
        fx = fy = 500.0
        cx = self.width / 2.0
        cy = self.height / 2.0

        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return info

    def run(self):
        """Main loop - optimized for max FPS"""
        if self.cap is None or not self.cap.isOpened():
            rospy.logerr("Camera not initialized")
            return

        rospy.loginfo("Publishing images... Press Ctrl+C to stop")

        while not rospy.is_shutdown():
            # Capture frame
            ret, frame = self.cap.read()

            if not ret or frame is None:
                rospy.logwarn_throttle(5.0, "Failed to capture frame")
                continue

            # If using GStreamer, format conversion already done in pipeline
            if not self.use_gstreamer:
                # Resize if needed (V4L2 backend)
                if frame.shape[1] != self.width or frame.shape[0] != self.height:
                    frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_LINEAR)

                # Convert to grayscale if requested
                if self.use_gray and len(frame.shape) == 3:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Determine encoding
            encoding = 'mono8' if self.use_gray else 'bgr8'

            # Create ROS message
            timestamp = rospy.Time.now()

            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding=encoding)
                img_msg.header.stamp = timestamp
                img_msg.header.frame_id = self.frame_id

                self.camera_info.header.stamp = timestamp

                # Publish
                self.image_pub.publish(img_msg)
                self.camera_info_pub.publish(self.camera_info)

            except Exception as e:
                rospy.logerr(f"Error publishing: {e}")

    def shutdown(self):
        """Cleanup"""
        rospy.loginfo("Shutting down camera publisher...")
        if self.cap is not None:
            self.cap.release()

if __name__ == '__main__':
    try:
        node = RealCameraPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")
