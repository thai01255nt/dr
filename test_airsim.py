# import airsim
# import numpy as np
# import time
# client = airsim.MultirotorClient(ip="127.0.0.1", port=41451)
# client.confirmConnection()
# lidar_name = "Lidar1"
# print("Đang lấy dữ liệu từ:", lidar_name)
# while True:
#     lidar_data = client.getLidarData(lidar_name=lidar_name)
#     if len(lidar_data.point_cloud) < 3:
#         print("⚠️ Không có dữ liệu LiDAR (point_cloud trống)")
#         time.sleep(0.5)
#         continue
#     points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
#     print(f"🟢 Nhận {points.shape[0]} điểm")
#     print("Vài điểm đầu tiên:\n", points[:5])
#     # time.sleep(1)
#!/usr/bin/env python3
import cosysairsim as airsim
import cv2
import numpy as np
import time

def main():
    # Connect to AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("Connected to AirSim")
    print("Press 'q' to quit\n")

    frame_count = 0
    start_time = time.time()

    try:
        while True:
            loop_start = time.time()

            # Get image using simGetImage (returns raw bytes, need to decode)
            t_get = time.time()
            image_bytes = client.simGetImage("downward_camera", airsim.ImageType.Scene)
            get_time = time.time() - t_get

            if image_bytes:
                # Convert to numpy
                t_process = time.time()

                # simGetImage returns compressed PNG bytes, decode it
                img1d = np.frombuffer(image_bytes, dtype=np.uint8)
                img_bgr = cv2.imdecode(img1d, cv2.IMREAD_COLOR)

                if img_bgr is not None:
                    process_time = time.time() - t_process

                    # Calculate FPS
                    frame_count += 1
                    elapsed = time.time() - start_time
                    fps = frame_count / elapsed if elapsed > 0 else 0

                    # Display info on frame
                    height, width = img_bgr.shape[:2]
                    cv2.putText(img_bgr, f"FPS: {fps:.1f}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(img_bgr, f"Size: {width}x{height}", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(img_bgr, f"Get: {get_time*1000:.1f}ms", (10, 90),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(img_bgr, f"Decode: {process_time*1000:.1f}ms", (10, 120),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    # Show
                    cv2.imshow("AirSim Camera", img_bgr)

                    # Print timing info to console (every 30 frames)
                    if frame_count % 30 == 0:
                        print(f"Frame {frame_count}: Get={get_time*1000:.1f}ms, Decode={process_time*1000:.1f}ms, FPS={fps:.1f}")

            # Check for quit (1ms wait to process key events)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        cv2.destroyAllWindows()
        elapsed = time.time() - start_time
        if frame_count > 0:
            print(f"\nFinal stats:")
            print(f"  Total frames: {frame_count}")
            print(f"  Average FPS: {frame_count / elapsed:.2f}")
            print(f"  Total time: {elapsed:.2f}s")

if __name__ == '__main__':
    main()