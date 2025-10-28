import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node("real_camera_publisher", anonymous=False)
    camera_id = rospy.get_param("~camera_id", "/dev/video11")

    # Mở camera với API V4L2
    cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)

    # Đặt định dạng và kích thước
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'NV12'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        rospy.logerr(f"Cannot open camera {camera_id}")
        return

    bridge = CvBridge()
    pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to read frame")
            continue

        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(msg)
        rate.sleep()

    cap.release()

if __name__ == "__main__":
    main()
