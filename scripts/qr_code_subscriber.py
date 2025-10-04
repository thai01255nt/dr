#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    cv2.imshow("QR Image", frame)
    cv2.waitKey(1)

def main():
    rospy.init_node("aruco_subscriber", anonymous=True)
    rospy.Subscriber("/qr_image", Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
