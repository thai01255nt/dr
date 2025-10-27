#!/usr/bin/env python3

"""
Generate ChArUco calibration board for camera calibration.
ChArUco = Chessboard + ArUco markers, more robust than pure checkerboard.
"""

import cv2
import numpy as np
import argparse
import os

def generate_charuco_board(
    squares_x=5,
    squares_y=7,
    square_length=0.04,  # meters (40mm)
    marker_length=0.02,  # meters (20mm)
    dictionary_name='DICT_6X6_250',
    output_file='charuco_board.png',
    dpi=300
):
    """
    Generate ChArUco board and save to file.

    Args:
        squares_x: Number of squares in X direction
        squares_y: Number of squares in Y direction
        square_length: Length of square side in meters
        marker_length: Length of ArUco marker side in meters
        dictionary_name: ArUco dictionary name
        output_file: Output filename (PNG or PDF)
        dpi: DPI for output image
    """

    # Get ArUco dictionary
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

    if dictionary_name not in aruco_dict_map:
        print(f"Error: Dictionary {dictionary_name} not found!")
        print(f"Available dictionaries: {list(aruco_dict_map.keys())}")
        return False

    # Create ArUco dictionary and ChArUco board
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_map[dictionary_name])
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y),
        square_length,
        marker_length,
        aruco_dict
    )

    # Calculate image size based on DPI
    # A4 size at 300 DPI = 2480 x 3508 pixels
    # We'll use a reasonable size based on board dimensions
    pixels_per_meter = dpi / 0.0254  # Convert DPI to pixels per meter

    board_width_meters = squares_x * square_length
    board_height_meters = squares_y * square_length

    img_width = int(board_width_meters * pixels_per_meter)
    img_height = int(board_height_meters * pixels_per_meter)

    # Add margins (20% on each side)
    margin_x = int(img_width * 0.2)
    margin_y = int(img_height * 0.2)

    total_width = img_width + 2 * margin_x
    total_height = img_height + 2 * margin_y

    print("=" * 60)
    print("ChArUco Board Configuration:")
    print("=" * 60)
    print(f"Board size: {squares_x} x {squares_y} squares")
    print(f"Square length: {square_length * 1000:.1f} mm")
    print(f"Marker length: {marker_length * 1000:.1f} mm")
    print(f"Dictionary: {dictionary_name}")
    print(f"Board dimensions: {board_width_meters * 1000:.1f} x {board_height_meters * 1000:.1f} mm")
    print(f"Image size: {total_width} x {total_height} pixels ({dpi} DPI)")
    print(f"Output file: {output_file}")
    print("=" * 60)

    # Generate board image
    board_img = board.generateImage((img_width, img_height), marginSize=0, borderBits=1)

    # Add white margins
    img_with_margin = np.ones((total_height, total_width), dtype=np.uint8) * 255
    img_with_margin[margin_y:margin_y+img_height, margin_x:margin_x+img_width] = board_img

    # Save image
    cv2.imwrite(output_file, img_with_margin)

    print(f"\n✓ ChArUco board saved to: {output_file}")
    print("\nCalibration instructions:")
    print("1. Print this image on A4/A3 paper")
    print("2. Measure the actual square size after printing")
    print("3. Mount on flat, rigid surface (cardboard, foam board, etc.)")
    print("4. Run calibration script with measured square size")
    print("=" * 60)

    return True

def main():
    parser = argparse.ArgumentParser(
        description='Generate ChArUco calibration board',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate default 5x7 board with 40mm squares
  python3 generate_charuco_board.py

  # Generate 7x10 board with 30mm squares
  python3 generate_charuco_board.py --squares 7 10 --square-size 30

  # Generate board with custom marker size
  python3 generate_charuco_board.py --square-size 40 --marker-size 25

  # Use different ArUco dictionary
  python3 generate_charuco_board.py --dict DICT_5X5_100
        """
    )

    parser.add_argument(
        '--squares',
        type=int,
        nargs=2,
        default=[5, 7],
        metavar=('X', 'Y'),
        help='Number of squares in X and Y direction (default: 5 7)'
    )

    parser.add_argument(
        '--square-size',
        type=float,
        default=40.0,
        metavar='MM',
        help='Square side length in millimeters (default: 40.0)'
    )

    parser.add_argument(
        '--marker-size',
        type=float,
        default=20.0,
        metavar='MM',
        help='ArUco marker side length in millimeters (default: 20.0, should be < square-size)'
    )

    parser.add_argument(
        '--dict',
        type=str,
        default='DICT_6X6_250',
        help='ArUco dictionary (default: DICT_6X6_250)'
    )

    parser.add_argument(
        '--output',
        type=str,
        default='charuco_board.png',
        help='Output filename (default: charuco_board.png)'
    )

    parser.add_argument(
        '--dpi',
        type=int,
        default=300,
        help='DPI for output image (default: 300)'
    )

    args = parser.parse_args()

    # Validate marker size
    if args.marker_size >= args.square_size:
        print("Error: Marker size must be smaller than square size!")
        return 1

    # Convert mm to meters
    square_length = args.square_size / 1000.0
    marker_length = args.marker_size / 1000.0

    # Generate board
    success = generate_charuco_board(
        squares_x=args.squares[0],
        squares_y=args.squares[1],
        square_length=square_length,
        marker_length=marker_length,
        dictionary_name=args.dict,
        output_file=args.output,
        dpi=args.dpi
    )

    return 0 if success else 1

if __name__ == '__main__':
    exit(main())
