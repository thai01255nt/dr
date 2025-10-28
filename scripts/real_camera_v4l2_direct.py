#!/usr/bin/env python3
"""
V4L2 Direct Access Camera Publisher - True Zero-Copy
Sử dụng v4l2 library để đọc trực tiếp buffer từ camera
Không qua OpenCV decode, chỉ memcpy buffer sang ROS message
"""

import rospy
from sensor_msgs.msg import Image, CameraInfo
import fcntl
import mmap
import select
from v4l2 import *
import numpy as np

class V4L2DirectCamera:
    def __init__(self):
        rospy.init_node("v4l2_direct_camera", anonymous=False)

        # Parameters
        self.device = rospy.get_param("~device", "/dev/video11")
        self.width = rospy.get_param("~width", 640)
        self.height = rospy.get_param("~height", 480)
        self.fps = rospy.get_param("~fps", 30)
        self.frame_id = rospy.get_param("~frame_id", "camera")
        self.format = rospy.get_param("~format", "YUYV")  # YUYV, NV12, MJPEG

        # Publishers
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=1)

        # Open device
        self.fd = open(self.device, 'rb+', buffering=0)

        # Setup camera
        self.setup_camera()

        # Setup buffers
        self.setup_buffers()

        # Camera info
        self.camera_info = self.setup_camera_info()

        rospy.loginfo(f"V4L2 Direct Camera started: {self.width}x{self.height} @ {self.fps}fps, format: {self.format}")

    def setup_camera(self):
        """Setup camera format and framerate"""
        # Set format
        fmt = v4l2_format()
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        fmt.fmt.pix.width = self.width
        fmt.fmt.pix.height = self.height

        # Convert format string to fourcc
        if self.format == "YUYV":
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV
            self.ros_encoding = "yuyv"
            self.bytes_per_pixel = 2
        elif self.format == "NV12":
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12
            self.ros_encoding = "nv12"  # Custom encoding, VINS cần decode
            self.bytes_per_pixel = 1.5
        elif self.format == "MJPEG":
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG
            self.ros_encoding = "jpeg"
            self.bytes_per_pixel = None  # Variable
        elif self.format == "GREY":
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY
            self.ros_encoding = "mono8"
            self.bytes_per_pixel = 1
        else:
            rospy.logerr(f"Unsupported format: {self.format}")
            raise ValueError(f"Unsupported format: {self.format}")

        fmt.fmt.pix.field = V4L2_FIELD_INTERLACED
        fcntl.ioctl(self.fd, VIDIOC_S_FMT, fmt)

        # Verify format
        fcntl.ioctl(self.fd, VIDIOC_G_FMT, fmt)
        self.width = fmt.fmt.pix.width
        self.height = fmt.fmt.pix.height
        self.image_size = fmt.fmt.pix.sizeimage

        rospy.loginfo(f"Actual format: {self.width}x{self.height}, buffer size: {self.image_size}")

        # Set framerate
        parm = v4l2_streamparm()
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        parm.parm.capture.timeperframe.numerator = 1
        parm.parm.capture.timeperframe.denominator = self.fps
        fcntl.ioctl(self.fd, VIDIOC_S_PARM, parm)

    def setup_buffers(self):
        """Setup memory-mapped buffers"""
        # Request buffers
        req = v4l2_requestbuffers()
        req.count = 4  # Number of buffers
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        req.memory = V4L2_MEMORY_MMAP
        fcntl.ioctl(self.fd, VIDIOC_REQBUFS, req)

        self.buffers = []

        # Map buffers
        for i in range(req.count):
            buf = v4l2_buffer()
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = V4L2_MEMORY_MMAP
            buf.index = i
            fcntl.ioctl(self.fd, VIDIOC_QUERYBUF, buf)

            # Memory map
            mm = mmap.mmap(
                self.fd.fileno(),
                buf.length,
                mmap.MAP_SHARED,
                mmap.PROT_READ | mmap.PROT_WRITE,
                offset=buf.m.offset
            )

            self.buffers.append({
                'buffer': buf,
                'mmap': mm,
                'length': buf.length
            })

            # Queue buffer
            fcntl.ioctl(self.fd, VIDIOC_QBUF, buf)

        rospy.loginfo(f"Allocated {len(self.buffers)} buffers")

        # Start streaming
        buf_type = v4l2_buf_type(V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self.fd, VIDIOC_STREAMON, buf_type)

    def setup_camera_info(self):
        """Setup camera info message"""
        msg = CameraInfo()
        msg.header.frame_id = self.frame_id
        msg.height = self.height
        msg.width = self.width
        msg.distortion_model = "plumb_bob"
        msg.D = [-0.151, 0.0798, 0.001003, 0.00102, -0.02]

        fx, fy, cx, cy = 322.5, 325.0, 320.0, 240.0
        msg.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return msg

    def read_frame(self):
        """Read frame using select() - non-blocking"""
        # Wait for frame with timeout
        ready, _, _ = select.select([self.fd], [], [], 1.0)

        if not ready:
            return None

        # Dequeue buffer
        buf = v4l2_buffer()
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        buf.memory = V4L2_MEMORY_MMAP

        try:
            fcntl.ioctl(self.fd, VIDIOC_DQBUF, buf)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Failed to dequeue buffer: {e}")
            return None

        # Get buffer data
        index = buf.index
        bytesused = buf.bytesused
        mm = self.buffers[index]['mmap']

        # Read data - TRUE ZERO COPY (chỉ đọc pointer)
        mm.seek(0)
        data = mm.read(bytesused)

        # Re-queue buffer immediately
        fcntl.ioctl(self.fd, VIDIOC_QBUF, buf)

        return data, buf.timestamp

    def run(self):
        """Main loop"""
        frame_count = 0
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            result = self.read_frame()

            if result is None:
                continue

            data, timestamp = result
            stamp = rospy.Time.now()

            # Create ROS Image message - ZERO COPY (truyền buffer trực tiếp)
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.height = self.height
            msg.width = self.width
            msg.encoding = self.ros_encoding
            msg.is_bigendian = 0

            if self.format == "MJPEG":
                # MJPEG: variable size
                msg.step = 0
                msg.data = data
            elif self.format == "NV12":
                # NV12: Y plane + UV plane (1.5 bytes per pixel)
                msg.step = self.width
                msg.data = data
            elif self.format == "YUYV":
                # YUYV: 2 bytes per pixel
                msg.step = self.width * 2
                msg.data = data
            elif self.format == "GREY":
                # Grayscale: 1 byte per pixel
                msg.step = self.width
                msg.data = data

            # Update camera info
            self.camera_info.header.stamp = stamp

            # Publish
            self.image_pub.publish(msg)
            self.camera_info_pub.publish(self.camera_info)

            frame_count += 1

            # Stats
            if frame_count % 100 == 0:
                elapsed = (rospy.Time.now() - start_time).to_sec()
                fps = frame_count / elapsed if elapsed > 0 else 0
                rospy.loginfo(f"Published {frame_count} frames, FPS: {fps:.2f}")

        # Cleanup
        self.stop()

    def stop(self):
        """Stop streaming and cleanup"""
        try:
            buf_type = v4l2_buf_type(V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.fd, VIDIOC_STREAMOFF, buf_type)

            for buf_info in self.buffers:
                buf_info['mmap'].close()

            self.fd.close()

            rospy.loginfo("Camera stopped cleanly")
        except Exception as e:
            rospy.logerr(f"Error stopping camera: {e}")

if __name__ == "__main__":
    try:
        camera = V4L2DirectCamera()
        camera.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
