#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import threading
from queue import Queue

class OptimizedCameraPublisher:
    def __init__(self):
        rospy.init_node("real_camera_publisher_optimized", anonymous=False)

        # Parameters
        self.camera_id = rospy.get_param("~camera_id", "/dev/video11")
        self.width = rospy.get_param("~width", 640)
        self.height = rospy.get_param("~height", 480)
        self.fps = rospy.get_param("~fps", 30)
        self.publish_color = rospy.get_param("~publish_color", False)  # False = grayscale (faster)
        self.frame_id = rospy.get_param("~frame_id", "camera")

        # Open camera with V4L2
        self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)

        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'NV12'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency

        if not self.cap.isOpened():
            rospy.logerr(f"Cannot open camera {self.camera_id}")
            return

        rospy.loginfo(f"Camera opened: {self.width}x{self.height} @ {self.fps} fps")

        # Publishers with queue_size=1 for minimal latency
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=1)

        # Bridge (reuse instance for efficiency)
        self.bridge = CvBridge()

        # Setup camera info
        self.camera_info_msg = self.setup_camera_info()

        # Statistics
        self.frame_count = 0
        self.start_time = rospy.Time.now()

        rospy.loginfo("Optimized Real Camera Publisher started")

    def setup_camera_info(self):
        """Setup camera info message"""
        msg = CameraInfo()
        msg.header.frame_id = self.frame_id
        msg.height = self.height
        msg.width = self.width
        msg.distortion_model = "plumb_bob"

        # Default distortion coefficients (calibrate for your camera)
        msg.D = [-0.151, 0.0798, 0.001003, 0.00102, -0.02]

        # Default intrinsic matrix (calibrate for your camera)
        fx, fy, cx, cy = 322.5, 325.0, 320.0, 240.0
        msg.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return msg

    def run(self):
        """Main loop - run as fast as camera provides frames"""
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()

            if not ret or frame is None:
                rospy.logwarn_throttle(1.0, "Failed to read frame")
                continue

            stamp = rospy.Time.now()

            # Convert to appropriate format
            if self.publish_color:
                # Publish BGR8
                encoding = "bgr8"
                image_data = frame
            else:
                # Publish MONO8 (faster for VINS/VIO)
                if len(frame.shape) == 3:
                    image_data = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                else:
                    image_data = frame
                encoding = "mono8"

            # Create ROS message - optimized method
            msg = self.bridge.cv2_to_imgmsg(image_data, encoding=encoding)
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id

            # Update camera info timestamp
            self.camera_info_msg.header.stamp = stamp

            # Publish
            self.image_pub.publish(msg)
            self.camera_info_pub.publish(self.camera_info_msg)

            self.frame_count += 1

            # Print stats every 100 frames
            if self.frame_count % 100 == 0:
                elapsed = (rospy.Time.now() - self.start_time).to_sec()
                avg_fps = self.frame_count / elapsed if elapsed > 0 else 0
                rospy.loginfo(f"Published {self.frame_count} frames, avg FPS: {avg_fps:.2f}")

        # Cleanup
        self.cap.release()

        # Final stats
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        if elapsed > 0:
            rospy.loginfo(f"Final stats: {self.frame_count} frames in {elapsed:.2f} seconds ({self.frame_count/elapsed:.2f} FPS)")

if __name__ == "__main__":
    try:
        publisher = OptimizedCameraPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
