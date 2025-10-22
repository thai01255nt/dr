#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class CameraSubscriberTest:
    def __init__(self):
        rospy.init_node('camera_subscriber_test', anonymous=True)

        self.bridge = CvBridge()

        # Statistics
        self.frame_count = 0
        self.start_time = time.time()
        self.last_print_time = time.time()
        self.fps = 0.0

        # Subscribe to camera topic
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        rospy.loginfo("Camera Subscriber Test started")
        rospy.loginfo("Subscribing to: /camera/image_raw")
        rospy.loginfo("Press 'q' to quit\n")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            if msg.encoding == "mono8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
                # Convert to BGR for display
                # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "bgr8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            elif msg.encoding == "rgb8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            else:
                rospy.logwarn(f"Unsupported encoding: {msg.encoding}")
                return

            # Update statistics
            self.frame_count += 1
            elapsed = time.time() - self.start_time
            self.fps = self.frame_count / elapsed if elapsed > 0 else 0

            # Calculate latency
            now = rospy.Time.now()
            latency = (now - msg.header.stamp).to_sec() * 1000  # ms

            # Display info on image
            cv2.putText(cv_image, f"FPS: {self.fps:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(cv_image, f"Size: {msg.width}x{msg.height}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(cv_image, f"Encoding: {msg.encoding}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(cv_image, f"Latency: {latency:.1f}ms", (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(cv_image, f"Frame: {self.frame_count}", (10, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Show image
            cv2.imshow("Camera Subscriber Test", cv_image)

            # Print to console every 30 frames
            if self.frame_count % 30 == 0:
                print(f"Frame {self.frame_count}: FPS={self.fps:.2f}, Latency={latency:.1f}ms, Size={msg.width}x{msg.height}")

            # Check for quit (1ms to not block)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rospy.signal_shutdown("User pressed 'q'")

        except Exception as e:
            rospy.logerr(f"Error in image callback: {e}")

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            cv2.destroyAllWindows()
            elapsed = time.time() - self.start_time
            if self.frame_count > 0:
                print(f"\nFinal statistics:")
                print(f"  Total frames: {self.frame_count}")
                print(f"  Average FPS: {self.frame_count / elapsed:.2f}")
                print(f"  Total time: {elapsed:.2f}s")

if __name__ == '__main__':
    try:
        node = CameraSubscriberTest()
        node.run()
    except rospy.ROSInterruptException:
        pass
