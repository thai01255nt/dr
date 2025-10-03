import cv2
import cv2.aruco as aruco

results = []

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("ERROR: Cannot open camera")
        return
    # Chọn dictionary ArUco (ví dụ 6x6 với 250 id)
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, parameters)
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        # detect aruco
        corners, ids, rejected = detector.detectMarkers(frame)
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in results:
                    results.append(marker_id)
                    print(f"Detected ArUco ID: {marker_id}")
        # chỉ show ảnh gốc, không overlay
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
