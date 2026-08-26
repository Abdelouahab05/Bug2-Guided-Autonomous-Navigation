import time
import cv2
import cv2.aruco as aruco

from config import (
    ARUCO_CAMERA_INDEX, ARUCO_DICT_NAME,
    ARUCO_SCAN_DURATION_SEC, ARUCO_FRAME_WIDTH, ARUCO_FRAME_HEIGHT
)


def scan_for_marker(duration_sec: float = None) -> int:

    duration_sec = duration_sec or ARUCO_SCAN_DURATION_SEC

    cap = cv2.VideoCapture(ARUCO_CAMERA_INDEX)
    if not cap.isOpened():
        print("[ARUCO] Could not access camera.")
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, ARUCO_FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ARUCO_FRAME_HEIGHT)

    dict_type = getattr(aruco, ARUCO_DICT_NAME)
    aruco_dict = aruco.getPredefinedDictionary(dict_type)
    aruco_params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, aruco_params)

    print(f"[ARUCO] Scanning for {duration_sec:.1f}s...")

    detections = {}
    start = time.time()

    try:
        while time.time() - start < duration_sec:
            ret, frame = cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detector.detectMarkers(gray)

            if ids is not None:
                for marker_id in ids.flatten():
                    detections[marker_id] = detections.get(marker_id, 0) + 1
    finally:
        cap.release()

    if not detections:
        print("[ARUCO] No marker detected.")
        return None

    # Most frequently seen marker across the scan window
    best_id = max(detections, key=detections.get)
    print(f"[ARUCO] Marker detected: ID {best_id} "
          f"({detections[best_id]} hits)")
    return int(best_id)