#!/usr/bin/env python3
"""
V4L2 Minimal Camera Publisher - Zero-Copy without v4l2 library
Sử dụng ctypes + fcntl để đọc trực tiếp từ V4L2 device
Không cần thư viện v4l2 (bug với Python 3.8)
"""

import rospy
from sensor_msgs.msg import Image, CameraInfo
import fcntl
import mmap
import select
import struct
import ctypes
import os

# V4L2 Constants
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_FIELD_ANY = 0

VIDIOC_QUERYCAP = 0x80685600
VIDIOC_S_FMT = 0xC0D05605
VIDIOC_G_FMT = 0xC0D05604
VIDIOC_REQBUFS = 0xC0145608
VIDIOC_QUERYBUF = 0xC0445609
VIDIOC_QBUF = 0xC044560F
VIDIOC_DQBUF = 0xC0445611
VIDIOC_STREAMON = 0x40045612
VIDIOC_STREAMOFF = 0x40045613
VIDIOC_S_PARM = 0xC0CC5616

# Pixel format FourCC
def v4l2_fourcc(a, b, c, d):
    return ord(a) | (ord(b) << 8) | (ord(c) << 16) | (ord(d) << 24)

V4L2_PIX_FMT_YUYV = v4l2_fourcc('Y', 'U', 'Y', 'V')
V4L2_PIX_FMT_MJPEG = v4l2_fourcc('M', 'J', 'P', 'G')
V4L2_PIX_FMT_GREY = v4l2_fourcc('G', 'R', 'E', 'Y')
V4L2_PIX_FMT_NV12 = v4l2_fourcc('N', 'V', '1', '2')

class V4L2MinimalCamera:
    def __init__(self):
        rospy.init_node("v4l2_minimal_camera", anonymous=False)

        # Parameters
        self.device = rospy.get_param("~device", "/dev/video11")
        self.width = rospy.get_param("~width", 640)
        self.height = rospy.get_param("~height", 480)
        self.fps = rospy.get_param("~fps", 30)
        self.frame_id = rospy.get_param("~frame_id", "camera")
        self.format_name = rospy.get_param("~format", "YUYV")

        # Publishers
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=1)

        # Format mapping
        format_map = {
            "YUYV": (V4L2_PIX_FMT_YUYV, "yuyv", 2),
            "MJPEG": (V4L2_PIX_FMT_MJPEG, "jpeg", None),
            "GREY": (V4L2_PIX_FMT_GREY, "mono8", 1),
            "NV12": (V4L2_PIX_FMT_NV12, "nv12", 1.5),
        }

        if self.format_name not in format_map:
            rospy.logerr(f"Unsupported format: {self.format_name}")
            raise ValueError(f"Unsupported format: {self.format_name}")

        self.pixel_format, self.ros_encoding, self.bytes_per_pixel = format_map[self.format_name]

        # Open device
        self.fd = os.open(self.device, os.O_RDWR)
        rospy.loginfo(f"Opened device: {self.device}")

        # Setup camera
        self.setup_format()
        self.setup_buffers()

        # Camera info
        self.camera_info = self.setup_camera_info()

        rospy.loginfo(f"V4L2 Camera ready: {self.width}x{self.height} @ {self.fps}fps, format: {self.format_name}")

    def setup_format(self):
        """Setup video format using ioctl"""
        # Format structure: v4l2_format
        # struct v4l2_format {
        #     __u32 type;        // V4L2_BUF_TYPE_VIDEO_CAPTURE
        #     struct v4l2_pix_format {
        #         __u32 width;
        #         __u32 height;
        #         __u32 pixelformat;
        #         __u32 field;
        #         __u32 bytesperline;
        #         __u32 sizeimage;
        #         ... (32 bytes reserved)
        #     } pix;
        # };

        fmt = bytearray(200)  # v4l2_format struct
        struct.pack_into('I', fmt, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into('IIIIII', fmt, 4,
            self.width,
            self.height,
            self.pixel_format,
            V4L2_FIELD_ANY,
            0,  # bytesperline
            0   # sizeimage
        )

        # Set format
        fcntl.ioctl(self.fd, VIDIOC_S_FMT, fmt)

        # Get actual format
        fcntl.ioctl(self.fd, VIDIOC_G_FMT, fmt)

        # Parse result
        self.width = struct.unpack_from('I', fmt, 4)[0]
        self.height = struct.unpack_from('I', fmt, 8)[0]
        bytesperline = struct.unpack_from('I', fmt, 20)[0]
        self.sizeimage = struct.unpack_from('I', fmt, 24)[0]

        rospy.loginfo(f"Format set: {self.width}x{self.height}, buffer size: {self.sizeimage}")

        # Set framerate
        # struct v4l2_streamparm
        parm = bytearray(204)
        struct.pack_into('I', parm, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE)
        # capability, capturemode, timeperframe.numerator, timeperframe.denominator
        struct.pack_into('IIII', parm, 4, 0, 0, 1, self.fps)

        try:
            fcntl.ioctl(self.fd, VIDIOC_S_PARM, parm)
        except Exception as e:
            rospy.logwarn(f"Could not set framerate: {e}")

    def setup_buffers(self):
        """Setup memory-mapped buffers"""
        # Request buffers
        # struct v4l2_requestbuffers
        reqbuf = bytearray(20)
        struct.pack_into('IIII', reqbuf, 0, 4, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP, 0)
        fcntl.ioctl(self.fd, VIDIOC_REQBUFS, reqbuf)

        count = struct.unpack_from('I', reqbuf, 0)[0]
        rospy.loginfo(f"Allocated {count} buffers")

        self.buffers = []

        # Map buffers
        for i in range(count):
            # Query buffer
            # struct v4l2_buffer
            buf = bytearray(88)
            struct.pack_into('III', buf, 0, i, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP)
            fcntl.ioctl(self.fd, VIDIOC_QUERYBUF, buf)

            # Get buffer info
            length = struct.unpack_from('I', buf, 16)[0]
            offset = struct.unpack_from('I', buf, 24)[0]

            # Memory map
            mm = mmap.mmap(
                self.fd,
                length,
                mmap.MAP_SHARED,
                mmap.PROT_READ | mmap.PROT_WRITE,
                offset=offset
            )

            self.buffers.append({
                'index': i,
                'mmap': mm,
                'length': length
            })

            # Queue buffer
            struct.pack_into('III', buf, 0, i, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP)
            fcntl.ioctl(self.fd, VIDIOC_QBUF, buf)

        # Start streaming
        buf_type = struct.pack('I', V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self.fd, VIDIOC_STREAMON, buf_type)

        rospy.loginfo("Streaming started")

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
        """Read frame from V4L2 device"""
        # Wait for frame
        ready, _, _ = select.select([self.fd], [], [], 1.0)

        if not ready:
            return None

        # Dequeue buffer
        buf = bytearray(88)
        struct.pack_into('III', buf, 0, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP)

        try:
            fcntl.ioctl(self.fd, VIDIOC_DQBUF, buf)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Failed to dequeue: {e}")
            return None

        # Get buffer info
        index = struct.unpack_from('I', buf, 0)[0]
        bytesused = struct.unpack_from('I', buf, 12)[0]

        # Read data from mmap
        mm = self.buffers[index]['mmap']
        mm.seek(0)
        data = mm.read(bytesused)

        # Re-queue buffer
        struct.pack_into('III', buf, 0, index, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP)
        fcntl.ioctl(self.fd, VIDIOC_QBUF, buf)

        return data

    def run(self):
        """Main loop"""
        frame_count = 0
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            data = self.read_frame()

            if data is None:
                continue

            stamp = rospy.Time.now()

            # Create ROS message
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.height = self.height
            msg.width = self.width
            msg.encoding = self.ros_encoding
            msg.is_bigendian = 0

            if self.format_name == "YUYV":
                msg.step = self.width * 2
            elif self.format_name == "GREY":
                msg.step = self.width
            elif self.format_name == "NV12":
                msg.step = self.width
            else:  # MJPEG
                msg.step = 0

            msg.data = data

            # Update camera info
            self.camera_info.header.stamp = stamp

            # Publish
            self.image_pub.publish(msg)
            self.camera_info_pub.publish(self.camera_info)

            frame_count += 1

            if frame_count % 100 == 0:
                elapsed = (rospy.Time.now() - start_time).to_sec()
                fps = frame_count / elapsed if elapsed > 0 else 0
                rospy.loginfo(f"Published {frame_count} frames, FPS: {fps:.2f}")

        self.stop()

    def stop(self):
        """Stop streaming and cleanup"""
        try:
            buf_type = struct.pack('I', V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.fd, VIDIOC_STREAMOFF, buf_type)

            for buf_info in self.buffers:
                buf_info['mmap'].close()

            os.close(self.fd)

            rospy.loginfo("Camera stopped cleanly")
        except Exception as e:
            rospy.logerr(f"Error stopping camera: {e}")

if __name__ == "__main__":
    try:
        camera = V4L2MinimalCamera()
        camera.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
