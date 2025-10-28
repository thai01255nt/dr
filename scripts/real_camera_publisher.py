#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
import cv2

class RealCameraPublisher:
    def __init__(self):
        rospy.init_node('real_camera_publisher', anonymous=False)

        # Parameters
        self.camera_id = rospy.get_param('~camera_id', 11)
        self.frame_id = rospy.get_param('~frame_id', 'camera')
        self.use_gray = rospy.get_param('~use_gray', False)
        self.use_compressed = rospy.get_param('~use_compressed', False)

        self.width = 640
        self.height = 480
        self.fps = 30

        # Publishers
        if self.use_compressed:
            self.image_pub = rospy.Publisher('/camera/image_raw/compressed', CompressedImage, queue_size=1)
        else:
            self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)

        self.camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=1)
        self.bridge = CvBridge()

        # Camera info
        self.camera_info = self.create_camera_info()

        # Initialize GStreamer pipeline
        self.cap = self.init_camera()

        rospy.on_shutdown(self.shutdown)

    def init_camera(self):
        """Use GStreamer for low-latency capture (OV13855)"""
        gst_str = (
            f"v4l2src device=/dev/video{self.camera_id} ! "
            f"video/x-raw, width={self.width}, height={self.height}, framerate={self.fps}/1 ! "
            "videoconvert ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

        cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            rospy.logerr(f"Cannot open camera /dev/video{self.camera_id}")
            return None

        rospy.loginfo("=" * 50)
        rospy.loginfo(f"Camera: /dev/video{self.camera_id} (OV13855)")
        rospy.loginfo(f"Resolution: {self.width}x{self.height} @ {self.fps}fps")
        rospy.loginfo(f"Mode: {'GRAYSCALE' if self.use_gray else 'BGR'}")
        rospy.loginfo(f"Compressed publish: {self.use_compressed}")
        rospy.loginfo("=" * 50)

        return cap

    def create_camera_info(self):
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.height = self.height
        info.width = self.width
        info.distortion_model = "plumb_bob"

        fx = fy = 500.0
        cx = self.width / 2.0
        cy = self.height / 2.0

        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def run(self):
        if not self.cap or not self.cap.isOpened():
            rospy.logerr("Camera not initialized")
            return

        rospy.loginfo("Publishing camera stream... Press Ctrl+C to stop")

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn_throttle(5.0, "Frame capture failed")
                continue

            if self.use_gray:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                encoding = 'mono8'
            else:
                encoding = 'bgr8'

            timestamp = rospy.Time.now()
            try:
                if self.use_compressed:
                    img_msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
                    img_msg.header.stamp = timestamp
                    img_msg.header.frame_id = self.frame_id
                else:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, encoding=encoding)
                    img_msg.header.stamp = timestamp
                    img_msg.header.frame_id = self.frame_id

                self.camera_info.header.stamp = timestamp
                self.image_pub.publish(img_msg)
                self.camera_info_pub.publish(self.camera_info)

            except Exception as e:
                rospy.logerr_throttle(2.0, f"Publish error: {e}")

    def shutdown(self):
        rospy.loginfo("Shutting down camera...")
        if self.cap:
            self.cap.release()


if __name__ == '__main__':
    try:
        node = RealCameraPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")
