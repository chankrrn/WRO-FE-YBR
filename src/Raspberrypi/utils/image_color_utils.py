import numpy as np
import cv2
from utils.enums import Color

HUE_MAX = 180       # OpenCV packs the 360-degree hue circle into 0-179
HUE_SEAM = HUE_MAX - 1

# Colors are authored in RGB: an anchor color you can eyedropper straight out of
# a frame, plus how far the hue may drift from it and the saturation/value band
# it has to land in. Only the hue comes from the RGB triple -- S and V stay
# explicit, because an RGB box cannot express bounds like GREEN's value ceiling
# or WHITE's "any hue, almost no saturation". A hue_tol of None means the hue is
# ignored entirely.
COLOR_SPECS = {
    #                  anchor RGB        hue_tol  saturation   value
    Color.BLUE:       ((0, 42, 255),        15,   (50, 255),  (50, 255)),
    Color.ORANGE:     ((255, 213, 0),       12,   (50, 255),  (50, 255)),
    Color.ALL_COLORS: ((255, 255, 255),   None,   (100, 255), (50, 255)),
    #Color.GREEN:     ((0, 255, 0),         10,   (90, 255),  (50, 205)),
    Color.GREEN:      ((70, 120, 60),         9,   (90, 230),  (20, 250)),
    Color.RED:        ((140, 30, 30), 6, (125, 210), (55, 200)),
    Color.PINK:       ((255, 0, 153),       12,   (135, 255), (75, 255)),
    Color.WHITE:      ((255, 255, 255),   None,   (0, 30),    (100, 255)),
}


def rgb_to_hue(rgb):
    """
    Convert an RGB triple to its OpenCV hue (0-179).

    I/O:
        rgb (tuple): (R, G, B), each 0-255.
        returns: int: the hue channel of that color in OpenCV's HSV encoding.
    """
    pixel = np.uint8([[list(rgb)]])
    return int(cv2.cvtColor(pixel, cv2.COLOR_RGB2HSV)[0][0][0])


def build_hsv_range(anchor_rgb, hue_tol, saturation, value):
    """
    Turn an RGB-authored color spec into the HSV bounds cv2.inRange wants.

    A hue window that crosses the 0/360-degree seam is returned with an upper
    hue above 179, which is the form calculate_color_mask splits back into two
    masks (it reads the overshoot as `upper - 179`).

    I/O:
        anchor_rgb (tuple): representative (R, G, B) for the color.
        hue_tol (int|None): allowed hue drift either side of the anchor, or None for any hue.
        saturation (tuple): (min, max) saturation, 0-255.
        value (tuple): (min, max) value, 0-255.
        returns: tuple[np.ndarray, np.ndarray]: the lower and upper HSV bounds.
    """
    s_low, s_high = saturation
    v_low, v_high = value
    if hue_tol is None:
        h_low, h_high = 0, HUE_SEAM
    else:
        hue = rgb_to_hue(anchor_rgb)
        h_low, h_high = hue - hue_tol, hue + hue_tol
        if h_low < 0:
            h_low += HUE_MAX
            h_high += HUE_SEAM
    return np.array([h_low, s_low, v_low]), np.array([h_high, s_high, v_high])


COLOR_RANGES = {color: build_hsv_range(*spec) for color, spec in COLOR_SPECS.items()}

class ImageColorUtils:
    @staticmethod
    def calculate_color_mask(hsv_img, color):
        """
        Generate a binary mask for the specified color in an HSV image.

        I/O:
            hsv_img (np.ndarray): The input HSV image.
            color (Color): The color to detect (must be a key in COLOR_RANGES).
            returns: np.ndarray: A binary mask where the specified color is white (255) and all else is black (0).
        """
        lower = COLOR_RANGES[color][0].copy()
        upper = COLOR_RANGES[color][1].copy()
        if upper[0] > 179:
            extra = upper[0] - 179
            upper[0] = 179
            mask1 = cv2.inRange(hsv_img, lower, upper)
            lower[0] = 0
            upper2 = np.array([extra, upper[1], upper[2]])
            mask2 = cv2.inRange(hsv_img, lower, upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv_img, lower, upper)
        return mask

    @staticmethod
    def is_color_at_point(img, pt, color):
        """
        img: HSV image
        pt: (x, y) point
        color: string, one of the keys in COLOR_RANGES
        return: bool, True if the color at pt matches the specified color
        """
        lower, higher = COLOR_RANGES[color]
        x, y = map(int, pt)
        hsv_value = img[y, x]
        pixel = np.uint8([[hsv_value]])
        return cv2.inRange(pixel, lower, higher)[0] == 255
    
    @staticmethod
    def find_color_from_pt(img, pt):
        """
        img: HSV image
        pt: (x, y) point
        
        return: color if a color is detected at pt, else None
        """
        print("find color")
        x, y = map(int, pt)
        hsv_value = img[x, y]
        pixel = np.uint8([[hsv_value]])
        for color, (lower, upper) in COLOR_RANGES.items():
            if cv2.inRange(pixel, lower, upper)[0] == 255:
                print(color)
                return color
        

    @staticmethod
    def find_color_from_rect(img, rect, color):
        x_center = rect[0][0]
        y_center = rect[0][1]
        pt = (x_center, y_center)
        return ImageColorUtils.is_color_at_point(img, pt, color)
    
    @staticmethod
    def is_rect_green(img, rect):
        return ImageColorUtils.find_color_from_rect(img, rect, Color.GREEN)