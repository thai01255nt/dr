#!/usr/bin/env python3

"""
Camera calibration using ChArUco board with OpenCV and ROS.
Calculates intrinsic matrix (K) and distortion coefficients.
"""

import cv2
import numpy as np
import argparse
import os
import yaml
from pathlib import Path
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

class CharucoCalibrator:
    def __init__(self,
                 squares_x=11,
                 squares_y=8,
                 square_length=0.04,
                 marker_length=0.02,
                 dictionary_name='DICT_6X6_250'):
        """
        Initialize ChArUco calibrator.

        Args:
            squares_x: Number of squares in X direction
            squares_y: Number of squares in Y direction
            square_length: Length of square side in meters
            marker_length: Length of ArUco marker side in meters
            dictionary_name: ArUco dictionary name
        """
        self.squares_x = squares_x
        self.squares_y = squares_y
        self.square_length = square_length
        self.marker_length = marker_length

        # ArUco dictionary mapping
        aruco_dict_map = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
            'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
            'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
        }

        # Create ArUco dictionary and ChArUco board
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_map[dictionary_name])
        self.board = cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            square_length,
            marker_length,
            self.aruco_dict
        )

        # Detector parameters
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        # Storage for calibration data
        self.all_charuco_corners = []
        self.all_charuco_ids = []
        self.image_size = None

    def detect_board(self, image):
        """
        Detect ChArUco board in image.

        Args:
            image: Input image (grayscale or color)

        Returns:
            charuco_corners, charuco_ids, image_with_markers
        """
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # Detect ArUco markers
        marker_corners, marker_ids, rejected = self.detector.detectMarkers(gray)

        # If at least one marker detected
        charuco_corners = None
        charuco_ids = None
        image_copy = image.copy()

        if marker_ids is not None and len(marker_ids) > 0:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(image_copy, marker_corners, marker_ids)

            # Interpolate ChArUco corners
            num_corners, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, self.board
            )

            # Draw ChArUco corners
            if charuco_corners is not None and len(charuco_corners) > 3:
                cv2.aruco.drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids)

        return charuco_corners, charuco_ids, image_copy

    def add_calibration_image(self, image):
        """
        Add image for calibration.

        Args:
            image: Calibration image

        Returns:
            success, num_corners_detected
        """
        charuco_corners, charuco_ids, _ = self.detect_board(image)

        if charuco_corners is not None and len(charuco_corners) > 3:
            self.all_charuco_corners.append(charuco_corners)
            self.all_charuco_ids.append(charuco_ids)

            if self.image_size is None:
                self.image_size = (image.shape[1], image.shape[0])

            return True, len(charuco_corners)

        return False, 0

    def calibrate(self):
        """
        Perform camera calibration.

        Returns:
            success, camera_matrix, dist_coeffs, rvecs, tvecs, calibration_error
        """
        if len(self.all_charuco_corners) < 3:
            print(f"Error: Need at least 3 good images, got {len(self.all_charuco_corners)}")
            return False, None, None, None, None, None

        print(f"\nCalibrating with {len(self.all_charuco_corners)} images...")

        # Calibrate camera
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            self.all_charuco_corners,
            self.all_charuco_ids,
            self.board,
            self.image_size,
            None,
            None
        )

        # Calculate reprojection error
        total_error = 0
        total_points = 0

        for i in range(len(self.all_charuco_corners)):
            # Project points
            projected_points, _ = cv2.projectPoints(
                self.board.getChessboardCorners()[self.all_charuco_ids[i].flatten()],
                rvecs[i],
                tvecs[i],
                camera_matrix,
                dist_coeffs
            )

            # Calculate error
            error = cv2.norm(self.all_charuco_corners[i], projected_points, cv2.NORM_L2)
            total_error += error ** 2
            total_points += len(self.all_charuco_corners[i])

        mean_error = np.sqrt(total_error / total_points)

        return ret, camera_matrix, dist_coeffs, rvecs, tvecs, mean_error

class ROSImageSubscriber:
    """ROS Image subscriber for calibration."""

    def __init__(self, topic_name='/camera/image_raw'):
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.subscriber = rospy.Subscriber(topic_name, Image, self.image_callback, queue_size=1)

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.latest_frame = cv_image
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def get_frame(self):
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

def calibrate_from_ros_topic(topic_name='/camera/image_raw',
                              output_file='camera_calibration.yaml',
                              num_images=20,
                              squares_x=5,
                              squares_y=7,
                              square_length=40.0,
                              marker_length=20.0,
                              dictionary_name='DICT_6X6_250'):
    """
    Calibrate camera by capturing images from ROS topic.

    Args:
        topic_name: ROS image topic name
        output_file: Output YAML file for calibration results
        num_images: Number of images to capture
        squares_x, squares_y: Board dimensions
        square_length: Square size in mm
        marker_length: Marker size in mm
        dictionary_name: ArUco dictionary
    """
    # Initialize ROS node
    rospy.init_node('charuco_calibration', anonymous=True)

    # Convert mm to meters
    square_length_m = square_length / 1000.0
    marker_length_m = marker_length / 1000.0

    # Initialize calibrator
    calibrator = CharucoCalibrator(
        squares_x=squares_x,
        squares_y=squares_y,
        square_length=square_length_m,
        marker_length=marker_length_m,
        dictionary_name=dictionary_name
    )

    # Initialize ROS subscriber
    rospy.loginfo(f"Subscribing to topic: {topic_name}")
    image_sub = ROSImageSubscriber(topic_name)

    # Wait for first image
    rospy.loginfo("Waiting for images...")
    while not rospy.is_shutdown() and image_sub.get_frame() is None:
        rospy.sleep(0.1)

    if rospy.is_shutdown():
        print("ROS shutdown before receiving images")
        return False

    print("=" * 70)
    print("ChArUco Camera Calibration (ROS)")
    print("=" * 70)
    print(f"Topic: {topic_name}")
    print(f"Board: {squares_x}x{squares_y}, Square: {square_length}mm, Marker: {marker_length}mm")
    print(f"Dictionary: {dictionary_name}")
    print(f"Target images: {num_images}")
    print("=" * 70)
    print("\nInstructions:")
    print("  - Press SPACE to capture image when board is clearly visible")
    print("  - Move board to different positions and angles")
    print("  - Press 'q' to quit early and calibrate with captured images")
    print("  - Press ESC to cancel")
    print("=" * 70)

    captured_count = 0

    while captured_count < num_images and not rospy.is_shutdown():
        frame = image_sub.get_frame()

        if frame is None:
            rospy.sleep(0.01)
            continue

        # Detect board
        charuco_corners, charuco_ids, display_img = calibrator.detect_board(frame)

        # Display info
        info_text = f"Captured: {captured_count}/{num_images}"
        cv2.putText(display_img, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        if charuco_corners is not None and len(charuco_corners) > 3:
            corner_text = f"Corners detected: {len(charuco_corners)}"
            cv2.putText(display_img, corner_text, (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_img, "Press SPACE to capture", (10, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display_img, "No board detected", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow('Calibration', display_img)

        key = cv2.waitKey(1) & 0xFF

        # Capture image with SPACE
        if key == ord(' '):
            if charuco_corners is not None and len(charuco_corners) > 3:
                success, num_corners = calibrator.add_calibration_image(frame)
                if success:
                    captured_count += 1
                    print(f"✓ Image {captured_count}/{num_images} captured ({num_corners} corners)")
                else:
                    print("✗ Failed to add image (need at least 4 corners)")
            else:
                print("✗ No board detected, please adjust camera position")

        # Quit early with 'q'
        elif key == ord('q'):
            if captured_count >= 3:
                print(f"\nQuitting early with {captured_count} images...")
                break
            else:
                print(f"\n✗ Need at least 3 images, currently have {captured_count}")

        # Cancel with ESC
        elif key == 27:
            print("\nCalibration cancelled")
            cv2.destroyAllWindows()
            return False

    cv2.destroyAllWindows()

    # Perform calibration
    print("\n" + "=" * 70)
    success, camera_matrix, dist_coeffs, rvecs, tvecs, error = calibrator.calibrate()

    if not success:
        print("✗ Calibration failed!")
        return False

    # Display results
    print("✓ Calibration successful!")
    print("=" * 70)
    print("Camera Matrix (K):")
    print(camera_matrix)
    print("\nDistortion Coefficients (k1, k2, p1, p2, k3):")
    print(dist_coeffs.ravel())
    print(f"\nReprojection Error: {error:.4f} pixels")
    print("=" * 70)

    # Extract parameters
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    k1, k2, p1, p2, k3 = dist_coeffs.ravel()

    # Save to YAML
    calibration_data = {
        'image_width': calibrator.image_size[0],
        'image_height': calibrator.image_size[1],
        'camera_matrix': {
            'fx': float(fx),
            'fy': float(fy),
            'cx': float(cx),
            'cy': float(cy),
        },
        'distortion_coefficients': {
            'k1': float(k1),
            'k2': float(k2),
            'p1': float(p1),
            'p2': float(p2),
            'k3': float(k3),
        },
        'camera_matrix_K': camera_matrix.tolist(),
        'distortion_coefficients_D': dist_coeffs.ravel().tolist(),
        'reprojection_error': float(error),
        'num_images': captured_count,
        'board_config': {
            'squares_x': squares_x,
            'squares_y': squares_y,
            'square_length_mm': square_length,
            'marker_length_mm': marker_length,
            'dictionary': dictionary_name,
        }
    }

    with open(output_file, 'w') as f:
        yaml.dump(calibration_data, f, default_flow_style=False)

    print(f"\n✓ Calibration saved to: {output_file}")
    print("\nFor ROS camera publisher, use these parameters:")
    print(f"  fx: {fx:.4f}")
    print(f"  fy: {fy:.4f}")
    print(f"  cx: {cx:.4f}")
    print(f"  cy: {cy:.4f}")
    print(f"  k1: {k1:.6f}")
    print(f"  k2: {k2:.6f}")
    print(f"  p1: {p1:.6f}")
    print(f"  p2: {p2:.6f}")
    print(f"  k3: {k3:.6f}")
    print("=" * 70)

    return True

def main():
    parser = argparse.ArgumentParser(
        description='Camera calibration using ChArUco board with ROS',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Calibrate using ROS topic with default settings
  python3 calibrate_camera_charuco.py

  # Calibrate with custom board size (11x8 board, 15mm squares)
  python3 calibrate_camera_charuco.py --squares 11 8 --square-size 15 --marker-size 10

  # Use custom ROS topic and output file
  python3 calibrate_camera_charuco.py --topic /my_camera/image --output my_calib.yaml

  # Capture more images for better accuracy
  python3 calibrate_camera_charuco.py --num-images 30
        """
    )

    parser.add_argument(
        '--topic',
        type=str,
        default='/camera/image_raw',
        help='ROS image topic name (default: /camera/image_raw)'
    )

    parser.add_argument(
        '--output',
        type=str,
        default='camera_calibration.yaml',
        help='Output YAML file (default: camera_calibration.yaml)'
    )

    parser.add_argument(
        '--num-images',
        type=int,
        default=20,
        help='Number of images to capture (default: 20)'
    )

    parser.add_argument(
        '--squares',
        type=int,
        nargs=2,
        default=[11, 8],
        metavar=('X', 'Y'),
        help='Number of squares in X and Y (default: 5 7)'
    )

    parser.add_argument(
        '--square-size',
        type=float,
        default=15,
        help='Square side length in mm (default: 40.0)'
    )

    parser.add_argument(
        '--marker-size',
        type=float,
        default=11,
        help='Marker side length in mm (default: 20.0)'
    )
            # 'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            # 'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            # 'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            # 'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            # 'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            # 'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
            # 'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            # 'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
            # 'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            # 'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
            # 'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
            # 'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
    parser.add_argument(
        '--dict',
        type=str,
        default='DICT_4X4_250',
        help='ArUco dictionary (default: DICT_6X6_250)'
    )

    args = parser.parse_args()

    # Validate
    if args.marker_size >= args.square_size:
        print("Error: Marker size must be smaller than square size!")
        return 1

    # Run calibration
    success = calibrate_from_ros_topic(
        topic_name=args.topic,
        output_file=args.output,
        num_images=args.num_images,
        squares_x=args.squares[0],
        squares_y=args.squares[1],
        square_length=args.square_size,
        marker_length=args.marker_size,
        dictionary_name=args.dict
    )

    return 0 if success else 1

if __name__ == '__main__':
    exit(main())

# python3 scripts/calibrate_camera_charuco.py --squares 11 8 --square-size 15 --marker-size 11 --num-images 25 --output camera_calibration.yaml
