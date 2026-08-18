"""
Manual test script for classes/object_solver.py

Modes:
    python test_object_solver.py               live camera loop, debug visualizer on, ESC to quit
    python test_object_solver.py --synthetic    generates a fake frame with a green + red box,
                                                 no camera/hardware required

The debug visualizer opens two windows: the camera frame with detected boxes
outlined, and a top-down "bird's eye" view of the robot with each detected
box plotted at its solved distance/bearing.
"""
import argparse
import time

import cv2
import numpy as np

from classes.object_solver import ObjectSolver
from utils.image_transform_utils import ImageTransformUtils


def hsv_to_bgr(h, s, v):
    pixel = np.uint8([[[h, s, v]]])
    return tuple(int(c) for c in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0][0])


def run_synthetic_check():
    solver = ObjectSolver(debug=True)
    width, height = solver.image_width, solver.image_height
    image = np.zeros((height, width, 3), dtype=np.uint8)

    # colors picked from the middle of the GREEN/RED HSV ranges in
    # utils/image_color_utils.py so they're reliably detected
    green_bgr = hsv_to_bgr(60, 200, 180)
    red_bgr = hsv_to_bgr(5, 150, 150)

    # small box near top-center -> far away; big box lower-left -> close
    cv2.rectangle(image, (300, 60), (340, 90), green_bgr, -1)
    cv2.rectangle(image, (150, 180), (230, 260), red_bgr, -1)

    hsv_image = ImageTransformUtils.bgr_to_hsv(image)
    objects = solver.detect(hsv_image, display_image=image)

    print(f"Detected {len(objects)} object(s):")
    for obj in objects:
        print(f"  {obj.color.value:>5}: distance={obj.distance_cm:6.1f}cm  bearing={obj.bearing_deg:6.1f}deg "
              f"forward={obj.forward_cm:6.1f}cm  lateral={obj.lateral_cm:6.1f}cm")

    print("\nPress any key on an OpenCV window to exit...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def run_live():
    from Raspberrypi.camera_manager import CameraManager

    camera_manager = CameraManager()
    solver = ObjectSolver(debug=True)
    camera_manager.start_camera()

    print("Running live object solver. Press ESC (with an OpenCV window focused) to quit.")
    try:
        while True:
            camera_manager.capture_image()
            camera_manager.transform_image()

            objects = solver.detect(camera_manager.hsv_image, display_image=camera_manager.display_image)
            for obj in objects:
                print(f"{obj.color.value:>5}: dist={obj.distance_cm}cm bearing={obj.bearing_deg}deg "
                      f"fwd={obj.forward_cm}cm lat={obj.lateral_cm}cm")

            key = cv2.waitKey(1)
            if key == 27:  # ESC
                break
            time.sleep(0.01)
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test ObjectSolver GREEN/RED box detection")
    parser.add_argument("--synthetic", action="store_true",
                         help="Run against a generated test frame instead of the live camera")
    args = parser.parse_args()

    if args.synthetic:
        run_synthetic_check()
    else:
        run_live()
