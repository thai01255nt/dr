import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node("real_camera_publisher", anonymous=False)
    camera_id = rospy.get_param("~camera_id", "/dev/video11")

    # Mở camera với API V4L2
    cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)

    # Thử dùng MJPEG trước (nhanh hơn NV12 do hardware decode)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Giảm latency

    if not cap.isOpened():
        rospy.logerr(f"Cannot open camera {camera_id}")
        return

    bridge = CvBridge()
    pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)

    # Không dùng rate.sleep() - chạy với tốc độ camera cung cấp
    frame_count = 0
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn_throttle(1.0, "Failed to read frame")
            continue

        # Convert BGR -> GRAY để giảm data size (VINS dùng gray)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        msg = bridge.cv2_to_imgmsg(gray, encoding="mono8")
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera"
        pub.publish(msg)

        frame_count += 1
        if frame_count % 100 == 0:
            elapsed = (rospy.Time.now() - start_time).to_sec()
            fps = frame_count / elapsed if elapsed > 0 else 0
            rospy.loginfo(f"FPS: {fps:.2f}")

    cap.release()

if __name__ == "__main__":
    main()
