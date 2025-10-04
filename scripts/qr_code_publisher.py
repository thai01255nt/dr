#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

results = []

def main():
    rospy.init_node("aruco_publisher", anonymous=True)
    pub = rospy.Publisher("/qr_image", Image, queue_size=10)
    bridge = CvBridge()

    cap = cv2.VideoCapture(11)
    if not cap.isOpened():
        rospy.logerr("ERROR: Cannot open camera")
        return

    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, parameters)

    rate = rospy.Rate(30)  # 30 Hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue

        # detect aruco markers
        corners, ids, rejected = detector.detectMarkers(frame)
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in results:
                    results.append(marker_id)
                    rospy.loginfo(f"Detected ArUco ID: {marker_id}")

        # publish image (raw, no overlay)
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(msg)

        rate.sleep()

    cap.release()

if __name__ == "__main__":
    main()

