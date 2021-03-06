'''
Utility and Helper Methods.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import inspect
from enum import Enum
import cv2

import debug

##---------##
## Logging ##
##---------##

class Levels(Enum):
    '''
    Log Level Documentation.
    '''
    DEBUG = (0, "\033[35mDEBUG\033[0m")
    INFO = (1, "\033[34mINFO\033[0m")
    WARN = (2, "\033[33mWARN\033[0m")
    ERROR = (3, "\033[31mERROR\033[0m")
    FATAL = (4, "\033[101;30mFATAL\033[0m")

@debug.funcperf
def log(msg, level=Levels.INFO):
    '''
    Print a message with additional debugging information.
    '''
    # Get the source file and line number
    current_frame = inspect.currentframe()
    frame = inspect.getouterframes(current_frame, 2)[-1]
    # Print a nice formatted message
    print(
        f"[\033[38;5;245m{frame.filename[2:]}\033[0m:"
        f"\033[32m{frame.lineno}\033[0m:"
        f"\033[38;5;245m{frame.function}\033[0m]"
        f"[{level.value[1]}]"
        f"{msg}")

##--------------##
## Flow Control ##
##--------------##

class Abort(Exception):
    '''
    Exception raised when the tracking flow is interrupted.
    '''
    def __init__(self, message, level):
        super(Abort, self).__init__(message)
        self.message = message
        self.level = level

##-----------------------##
## OpenCV Helper Methods ##
##-----------------------##

@debug.funcperf
def crop(image, x1, y1, x2, y2):
    '''
    Crops an image using normalized (0..1) coordinates.
    '''
    height, width = image.shape[0:2]
    x1 = int(x1 * width)
    y1 = int(y1 * height)
    x2 = int(x2 * width)
    y2 = int(y2 * height)
    return image[y1:y2, x1:x2]

@debug.funcperf
def scale(image, factor):
    '''
    Scales an image by the given factor, preserving aspect ratio
    '''
    width = int(image.shape[1] * factor)
    height = int(image.shape[0] * factor)
    return cv2.resize(image, (width, height))

@debug.funcperf
def draw_rectangle(image, pt1, pt2, color, thickness=1):
    '''
    Draw a rectangle using normalized (0..1) coordinates.
    Arguments `pt1` and `pt2` are (x,y) tuples.
    '''
    x1 = int(pt1[0] * image.shape[1])
    y1 = int(pt1[1] * image.shape[0])
    x2 = int(pt2[0] * image.shape[1])
    y2 = int(pt2[1] * image.shape[0])
    cv2.rectangle(img=image, pt1=(x1, y1), pt2=(x2, y2), color=color, thickness=thickness)

@debug.funcperf
def draw_overlay(base, overlay, point, newimage=False, normcoords=True):
    '''
    Overlay `overlay` onto `base` at point `point`.

    Normally this will modify `base`, but if `newimage` is set to `True`, `base`
    is not modified, and the modified image is returned. If `normcoords` is set
    to `True`, the method accepts normalized (0..1) coordinates, otherwise it
    uses an absolute overlay.
    '''
    # Unpack arguments (Pylint made me do this 😡)
    x = point[0]
    y = point[1]
    # Edge case handling for a black and white overlay
    if len(overlay.shape) == 2:
        overlay = cv2.applyColorMap(
            cv2.convertScaleAbs(overlay, alpha=0.03),
            cv2.COLORMAP_RAINBOW)
    # Denormalize coordinates
    if normcoords:
        x = base.shape[1] * x
        y = base.shape[0] * y
    # Overlay the image
    width = overlay.shape[1]
    height = overlay.shape[0]
    # Make sure the overlay remains within bounds for the base image
    base_width = base.shape[1]
    base_height = base.shape[0]
    over_x = max(0, (x + width) - base_width)
    over_y = max(0, (y + height) - base_height)
    if over_x > 0 or over_y > 0:
        overlay = overlay[0:width - over_x, 0:height - over_y]
        width -= over_x
        height -= over_y
    # Overlay
    if newimage:
        base = base.copy()
    base[y:y+height, x:x+width] = overlay
    return base

@debug.funcperf
def draw_line(image, pt1, pt2, color=(0, 0, 0), thickness=1):
    '''
    Draw a line using normalized coordinates.
    Arguments `pt1` and `pt2` are (x,y) tuples.
    '''
    x1 = int(pt1[0] * image.shape[1])
    y1 = int(pt1[1] * image.shape[0])
    x2 = int(pt2[0] * image.shape[1])
    y2 = int(pt2[1] * image.shape[0])
    cv2.line(image, (x1, y1), (x2, y2), (color), thickness)

@debug.funcperf
def sample(image, x, y):
    '''
    Return a sample of the target image from the normalized coordinates (x, y).
    '''
    x = int(image.shape[1] * x)
    y = int(image.shape[0] * y)
    return image[y, x]

@debug.funcperf
def normalize_coordinates(image, x, y):
    '''
    Convert a set of coordinates from image coordinates to normalized (0..1)
    ones.
    '''
    return (x/image.shape[1], y/image.shape[0])
