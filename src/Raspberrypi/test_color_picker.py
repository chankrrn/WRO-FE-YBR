"""
Hover/click pixels on a camera frame to read off their RGB and HSV values -
for picking anchor colors and tuning the ranges in utils/image_color_utils.py.

Modes:
    python test_color_picker.py                 live camera, ESC to quit
    python test_color_picker.py --image shot.png a saved frame instead of the camera

Hover shows the pixel under the cursor in the window; click prints it to the
terminal as an (R, G, B) tuple, ready to paste into COLOR_SPECS.
"""
import argparse

import cv2

from utils.image_transform_utils import ImageTransformUtils

WINDOW = "Color Picker"


def make_callback(get_frame):
    def on_mouse(event, x, y, flags, userdata):
        frame = get_frame()
        if frame is None or not (0 <= y < frame.shape[0] and 0 <= x < frame.shape[1]):
            return
        b, g, r = (int(c) for c in frame[y, x])
        if event == cv2.EVENT_LBUTTONDOWN:
            hue, sat, val = (int(c) for c in ImageTransformUtils.bgr_to_hsv(frame)[y, x])
            print(f"({x},{y})  RGB=({r},{g},{b})  HSV=({hue},{sat},{val})")
    return on_mouse


def annotate(frame, x, y):
    if not (0 <= y < frame.shape[0] and 0 <= x < frame.shape[1]):
        return frame
    b, g, r = (int(c) for c in frame[y, x])
    shown = frame.copy()
    label = f"RGB({r},{g},{b})"
    cv2.putText(shown, label, (min(x + 12, shown.shape[1] - 160), max(y - 12, 15)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(shown, label, (min(x + 12, shown.shape[1] - 160), max(y - 12, 15)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return shown


def run_image(path):
    frame = cv2.imread(path)
    if frame is None:
        raise SystemExit(f"Could not read image: {path}")

    cursor = {"x": 0, "y": 0}
    click = make_callback(lambda: frame)

    def on_mouse(event, x, y, flags, userdata):
        cursor["x"], cursor["y"] = x, y
        click(event, x, y, flags, userdata)

    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, on_mouse)

    print("Hover to preview, click to print RGB/HSV. Press ESC to quit.")
    while True:
        cv2.imshow(WINDOW, annotate(frame, cursor["x"], cursor["y"]))
        if cv2.waitKey(30) == 27:
            break
    cv2.destroyAllWindows()


def run_live():
    from Raspberrypi.camera_manager import CameraManager

    camera_manager = CameraManager()
    camera_manager.start_camera()

    cursor = {"x": 0, "y": 0}
    cv2.namedWindow(WINDOW)

    def on_mouse(event, x, y, flags, userdata):
        cursor["x"], cursor["y"] = x, y
        make_callback(lambda: camera_manager.cropped_image)(event, x, y, flags, userdata)

    cv2.setMouseCallback(WINDOW, on_mouse)

    print("Hover to preview, click to print RGB/HSV. Press ESC (window focused) to quit.")
    try:
        while True:
            camera_manager.capture_image()
            camera_manager.transform_image()
            if camera_manager.cropped_image is None:
                continue
            cv2.imshow(WINDOW, annotate(camera_manager.cropped_image, cursor["x"], cursor["y"]))
            if cv2.waitKey(30) == 27:
                break
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pick RGB/HSV pixel values off a camera frame")
    parser.add_argument("--image", default=None, help="inspect a saved image instead of the live camera")
    args = parser.parse_args()

    if args.image:
        run_image(args.image)
    else:
        run_live()
