'''
Tracker Core.

This is the main component of the eye tracking system, where all of the
processing and derivation operations take place. In order to effectively
implement it into your application, there are some things you need to
know about.

Configuration:
| Option                      | Description                               |
|-----------------------------|-------------------------------------------|
| face_detection_scale_factor | Increases performance by downscaling the  |
|                             | source image before the face detection    |
|                             | algorithm is applied. This reduces the    |
|                             | maximum distance from the camera the      |
|                             | operator will be detected at.             |
|-----------------------------|-------------------------------------------|
| enable_prediction           | Enables or disables the tensorflow based  |
|                             | gaze prediction stage.                    |
|-----------------------------|-------------------------------------------|
| debug.create_debug_images   | Duplicates the source and processed       |
|                             | images and allows the drawing of debug    |
|                             | information. This option must be set to   |
|                             | `True` to enable the following debug.*    |
|                             | options.                                  |
|-----------------------------|-------------------------------------------|
| debug.draw_facial_features  | Overlays the result of the 68 point       |
|                             | facial feature detection algorithm onto   |
|                             | the debug image.                          |
|-----------------------------|-------------------------------------------|
| debug.draw_eye_boxes        | Draws boxes around the operators eyes.    |
|-----------------------------|-------------------------------------------|
| debug.show_processed_eyes   | Creates a new window to show the result   |
|                             | of the eye extraction and processing      |
|                             | algorithm.                                |
|-----------------------------|-------------------------------------------|
| debug.show_final            | Creates a new window to show the debug    |
|                             | images after each invocation of           |
|                             | `loop_once()`.                            |
|-----------------------------|-------------------------------------------|

If you're integrating this into another application, you should make sure
that all of the `debug.*` options are set to `False`. If you're trying to
train a model, or don't need the gaze estimates, you should set
`enable_prediction` to `False`.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import os
import pathlib
import math

import dlib
import pyrealsense2 as rs
import numpy as np
import cv2
import tensorflow as tf

from . import debug
from . import util

# Tracker data
data = {
    "status": "NOT_LOADED",
    "result": {
        # Populated by loop_once and actions
    }
}
# Configuration Options
config = {
    "face_detection_scale_factor": 0.25,
    "enable_prediction": True,
    "debug": {
        "create_debug_images": True,
        "draw_facial_features": True,
        "daw_eye_boxes": True,
        "show_processed_eyes": True,
        "show_final": True
    }
}

# Project Directory
DIR_PATH = os.path.dirname(os.path.realpath(__file__))

##------------------##
## Control Commands ##
##------------------##

def command_set_laserpower(power):
    '''
    Sets the realsense laser power to the specified value.
    '''
    power_range = data['rs_depthsensor'].get_option_range(rs.option.laser_power)
    if power == "default":
        util.log(f"Laser power set to default [{power_range.default}]", level=util.Levels.DEBUG)
        data['rs_depthsensor'].set_option(rs.option.laser_power, power_range.default)
        return
    if power_range.min > int(power) > power_range.max:
        util.log(f"Laser power value {power} is out of range [{power_range.min}-{power_range.max}]", level=util.Levels.ERROR)
        return
    util.log(f"Laser power changed to {power}", level=util.Levels.DEBUG)
    data['rs_depthsensor'].set_option(rs.option.laser_power, int(power))

##-----------##
## Main Loop ##
##-----------##

@debug.funcperf
def loop_once():
    '''
    Runs a single iteration of the processing loop.
    '''
    if data['status'] != "READY":
        util.log(f"Not Looping - State is not READY ({data['status']})", level=util.Levels.ERROR)
        return
    data['result'] = { # Reset result
        "valid": True,
        "face_depth": 0,
        "face_x": 0,
        "face_y": 0,
        "face_angle": 0,
        "depth_diff": 0,
        "left_eye": {
            "angle": 0,
            "x": 0,
            "y": 0
        },
        "right_eye": {
            "angle": 0,
            "x": 0,
            "y": 0
        },
        "gaze_prediction": {
            "x": 0,
            "y": 0
        }
    }
    try:
        acquire_image()
        preprocess_images()
        process_face()
        debug_draw_features()
        extract_eyes()
        estimate_screen_pos()
    except util.Abort as e:
        util.log(f"Processing aborted: {e.message}", e.level)
        data['result']['valid'] = False

    if config['debug']['show_final']:
        cv2.imshow("Debug: Color", data['color_frame_debug'])
        cv2.imshow("Debug: Depth", data['depth_frame_debug'])
        cv2.waitKey(delay=1)

    return data['result']

##----------------------##
## Main Process Methods ##
##----------------------##

@debug.funcperf
def init():
    '''
    Initialize the tracker core.
    '''
    util.log("\033[1mInitializing Tracker Core...\033[0m")
    data['status'] = "INITIALIZING"

    util.log("Initializing RealSense camera")
    data['rs_pipeline'] = rs.pipeline()
    data['rs_config'] = rs.config()
    data['rs_config'].enable_stream(rs.stream.color, width=1920, height=1080, format=rs.format.rgb8)
    data['rs_config'].enable_stream(rs.stream.depth, width=1280, height=720, format=rs.format.z16)
    data['rs_profile'] = data['rs_pipeline'].start(data['rs_config'])
    data['rs_device'] = data['rs_profile'].get_device()
    data['rs_depthsensor'] = data['rs_device'].query_sensors()[0]

    util.log("Initializing Face Detector")
    data['face_detector'] = dlib.get_frontal_face_detector()
    predictor_path = str(pathlib.Path(__file__).parent.absolute()) + \
        "/shape_predictor_68_face_landmarks.dat"
    if not os.path.exists(predictor_path):
        util.log("Unable to locate face prediction model - see README.md", level=util.Levels.ERROR)
        data['status'] = "ERROR"
        return
    data['face_predictor'] = dlib.shape_predictor(predictor_path)

    if config['enable_prediction']:
        util.log("Loading TensorFlow model...")
        data['tf_model'] = tf.keras.models.load_model(f"{DIR_PATH}/model")

    util.log("Tracker Core Initialization Complete!")
    data['status'] = "READY"


@debug.funcperf
def acquire_image():
    '''
    Attempts to acquire a new set of images from the RealSense camera.
    '''
    if data['status'] != "READY":
        raise util.Abort("Unable to acquire image - system not ready", util.Levels.ERROR)
    # Wait until both frames are ready
    while True:
        frames = data['rs_pipeline'].wait_for_frames()
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        if depth is not None:
            break
    data['color_raw'] = color
    data['depth_raw'] = depth

@debug.funcperf
def preprocess_images():
    '''
    The RGB and Depth frames generated by the camera are not immediately
    suitable for use within the algorithm because of differences in color and
    alignment. This method crops the RGB frame to align with the Depth frame,
    and generates a colorized depth frame useful for debugging.
    '''
    if data['status'] != "READY":
        util.log("Unable to acquire image - system not ready")
        return
    depth = np.asanyarray(data['depth_raw'].get_data())
    color = np.asanyarray(data['color_raw'].get_data())
    data['color_frame'] = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
    data['depth_frame'] = util.crop(depth, 0.13, 0.14, 0.84, 0.85)
    depth = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
    data['depth_color'] = util.crop(depth, 0.13, 0.14, 0.84, 0.85)
    if config['debug']['create_debug_images']:
        data['color_frame_debug'] = data['color_frame'].copy()
        data['depth_frame_debug'] = data['depth_color'].copy()

@debug.funcperf
def process_face():
    '''
    Use `dlib` to find faces in the frame. If no face is found, processing will
    stop after this function. If multiple faces are found, only one will be
    used. TODO: Find a way to determine which face will be selected.
    '''
    gray_image = cv2.cvtColor(data['color_frame'], cv2.COLOR_BGR2GRAY)
    scaled_image = util.scale(gray_image, config['face_detection_scale_factor'])
    faces = data['face_detector'](scaled_image)
    if len(faces) == 0:
        raise util.Abort("No faces found in frame", util.Levels.WARN)
    face = faces[0]
    # Get non-normalized face coordinates (offset scale factor)
    face = type(face)(
        int(face.left()   / config["face_detection_scale_factor"]),
        int(face.top()    / config["face_detection_scale_factor"]),
        int(face.right()  / config["face_detection_scale_factor"]),
        int(face.bottom() / config["face_detection_scale_factor"])
    )
    # Get normalized face coordinates
    data['face_normalized'] = (
        face.left()   / gray_image.shape[1],
        face.top()    / gray_image.shape[0],
        face.right()  / gray_image.shape[1],
        face.bottom() / gray_image.shape[0])
    # Find explicit facial features
    raw_features = data['face_predictor'](image=gray_image, box=face)
    features = np.zeros((68, 3))
    for i in range(0, 68):
        feature = raw_features.part(i)
        x = feature.x / gray_image.shape[1]
        y = feature.y / gray_image.shape[0]
        depth = util.sample(data['depth_frame'], x, y)
        features[i] = [x, y, depth]
    data['named_features'] = {
        "jaw": features[0:17],
        "right_brow": features[17:22],
        "left_brow": features[22:27],
        "nose": features[27:36],
        "right_eye": features[36:42],
        "left_eye": features[42:48],
        "outer_mouth": features[48:60],
        "inner_mouth": features[60:68]
    }
    # Find derived facial features
    def derive_avg(points):
        np_points = np.asanyarray(points)
        avg_x = sum(np_points[:, 0]) / len(points)
        avg_y = sum(np_points[:, 1]) / len(points)
        avg_d = sum(np_points[:, 2]) / len(points)
        sam_d = util.sample(data['depth_frame'], avg_x, avg_y)
        return [avg_x, avg_y, avg_d, sam_d]
    data['named_features']['glabella'] = derive_avg(np.concatenate((
        [data['named_features']['left_brow'][0]],
        [data['named_features']['right_brow'][-1]])))
    data['named_features']['left_cheek'] = derive_avg(np.concatenate((
        data['named_features']['jaw'][0:5],
        data['named_features']['nose'][4:8])))
    data['named_features']['right_cheek'] = derive_avg(np.concatenate((
        data['named_features']['jaw'][12:17],
        data['named_features']['nose'][4:8])))
    data['named_features']['chin'] = derive_avg(
        data['named_features']['jaw'][4:13])
    data['named_features']['left_eye_c'] = derive_avg(
        data['named_features']['left_eye'])
    data['named_features']['right_eye_c'] = derive_avg(
        data['named_features']['right_eye'])
    # Get result position components
    data['result']['face_depth'] = data['named_features']['glabella'][3]
    data['result']['face_x'] = data['named_features']['glabella'][0]
    data['result']['face_y'] = data['named_features']['glabella'][1]
    # Calculate angular information
    dx = data['named_features']['left_eye_c'][0] - data['named_features']['right_eye_c'][0]
    dy = data['named_features']['left_eye_c'][1] - data['named_features']['right_eye_c'][1]
    data['result']['face_angle'] = math.atan2(dy, dx)
    data['result']['depth_diff'] = data['named_features']['right_eye_c'][2] - data['named_features']['left_eye_c'][2]


@debug.funcperf
def debug_draw_features():
    '''
    This debug method draws facial features over the image and displays it.
    '''
    if not config['debug']['draw_facial_features']:
        return
    # Create copy of face image
    color = (0, 255, 0)
    thickness = 2
    def draw(name, start):
        for i in range(start, len(data['named_features'][name])):
            pt1 = data['named_features'][name][i-1][0:2]
            pt2 = data['named_features'][name][i][0:2]
            util.draw_line(data['color_frame_debug'], pt1, pt2, color, thickness)
            util.draw_line(data['depth_frame_debug'], pt1, pt2, color, thickness)
    draw('jaw', 1)
    draw('right_brow', 1)
    draw('left_brow', 1)
    draw('nose', 1)
    draw('right_eye', 0)
    draw('left_eye', 0)
    draw('outer_mouth', 0)
    draw('inner_mouth', 0)
    # Draw special features
    color = (0, 0, 255)
    thickness = 1
    def draw_x(name):
        x = data['named_features'][name][0]
        y = data['named_features'][name][1]
        pt1 = [x-0.01, y-0.01]
        pt2 = [x+0.01, y+0.01]
        pt3 = [x-0.01, y+0.01]
        pt4 = [x+0.01, y-0.01]
        util.draw_line(data['color_frame_debug'], pt1, pt2, color, thickness)
        util.draw_line(data['color_frame_debug'], pt3, pt4, color, thickness)
        util.draw_line(data['depth_frame_debug'], pt1, pt2, color, thickness)
        util.draw_line(data['depth_frame_debug'], pt3, pt4, color, thickness)
    draw_x('glabella')
    draw_x('left_cheek')
    draw_x('right_cheek')
    draw_x('chin')
    draw_x('left_eye_c')
    draw_x('right_eye_c')

@debug.funcperf
def extract_eyes():
    '''
    Removes eyes from the operators head. They probably weren't using them.
    '''
    @debug.funcperf
    def geteye(eye):
        # Get extents of eye box w/ padding
        min_x = min([a[0] for a in data['named_features'][eye]]) - 0.0
        max_x = max([a[0] for a in data['named_features'][eye]]) + 0.0
        min_y = min([a[1] for a in data['named_features'][eye]]) - 0.0
        max_y = max([a[1] for a in data['named_features'][eye]]) + 0.0
        # Bound Check
        min_x = util.bounds(0, min_x, 1)
        max_x = util.bounds(0, max_x, 1)
        min_y = util.bounds(0, min_y, 1)
        max_y = util.bounds(0, max_y, 1)
        # Offsets
        cv_offset = (min_x, min_y)
        cv_scale = (
            1 / (max_x - min_x),
            1 / (max_y - min_y))
        # Transform points into the cropped eye space
        points = [
            (
                (a[0] - cv_offset[0]) * cv_scale[0],
                (a[1] - cv_offset[1]) * cv_scale[1]
            ) for a in data['named_features'][eye]]
        invert_points = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)] + points + [points[0]]+ [(0, 0)]
        # Extract the eye from the image
        extracted = util.crop(data['color_frame'], min_x, min_y, max_x, max_y).copy()
        # Calculate eye angle
        # This math is off a bit, it doesn't account for the eye image aspect ratio
        a = points[0]
        b = points[3]
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        angle = math.atan2(dy, dx)
        # Mask the area surrounding the eye
        eye_mask_color = (0, 255, 0)
        #cv2.imwrite("/tmp/debug0.png", extracted)
        util.draw_poly(extracted, invert_points, color=eye_mask_color)
        # Get eye averages for pupil location. The eye average for pupil
        # location system works in two passes. The first pass determines the
        # average gray level for pixels in the unmasked area of the image. The
        # second pass tests only for values that are unmasked *and* lower than
        # the average determined in step 1, and calculates the average location
        # of those pixels, which is assumed to be roughly equivalent to the
        # center of the pupil/iris (or at the very least has a linear
        # relationship to the actual center point)
        #cv2.imwrite("/tmp/debug1.png", extracted)
        extracted_g = cv2.cvtColor(extracted, cv2.COLOR_BGR2GRAY)
        count = 0
        graysum = 0
        def np_tuple_comp(a, b):
            return a[0] == b[0] and a[1] == b[1] and a[2] == b[2]
        for y in range(0, extracted.shape[0]): # Pass 1
            for x in range(0, extracted.shape[1]):
                if not np_tuple_comp(extracted[y, x], eye_mask_color):
                    count += 1
                    graysum += extracted_g[y, x]
        if count > 0:
            avg = graysum / count
        else:
            raise util.Abort(f"Eye {eye} has 0 unmasked pixels (A)", level=util.Levels.WARN)
        x_sum = 0
        y_sum = 0
        count = 0
        for y in range(0, extracted.shape[0]): # Pass 2
            for x in range(0, extracted.shape[1]):
                if not np_tuple_comp(extracted[y, x], eye_mask_color):
                    if extracted_g[y, x] < avg:
                        x_sum += x
                        y_sum += y
                        count += 1
                    #    extracted[y, x] = (0, 0, 0)
                    #else:
                    #    extracted[y, x] = (255, 255, 255)
        #cv2.imwrite("/tmp/debug2.png", extracted)
        if count > 0:
            x_avg = x_sum / count / extracted.shape[1]
            y_avg = y_sum / count / extracted.shape[0]
        else:
            raise util.Abort(f"Eye {eye} has 0 unmasked pixels (B)", level=util.Levels.WARN)
        util.draw_line(extracted, (0, 0), (x_avg, y_avg), color=(255, 0, 255))
        #cv2.imwrite("/tmp/debug3.png", extracted)
        # Debug
        if config['debug']['daw_eye_boxes']:
            util.draw_rectangle(data['color_frame_debug'], (min_x, min_y), (max_x, max_y), (255, 0, 0))
            util.draw_rectangle(data['depth_frame_debug'], (min_x, min_y), (max_x, max_y), (255, 0, 0))
        if config['debug']['show_processed_eyes']:
            cv2.imshow(f"extracted_{eye}", extracted)
        return {
            "angle": angle,
            "x": x_avg,
            "y": y_avg
        }
    left = geteye("left_eye")
    right = geteye("right_eye")
    data['result']['left_eye']['angle'] = left['angle']
    data['result']['left_eye']['x'] = left['x']
    data['result']['left_eye']['y'] = left['y']
    data['result']['right_eye']['angle'] = right['angle']
    data['result']['right_eye']['x'] = right['x']
    data['result']['right_eye']['y'] = right['y']

@debug.funcperf
def estimate_screen_pos():
    '''
    Perform gaze target estimation using the default model.
    '''
    prediction = data['tf_model'].predict(
        [[
            float(data['result']['face_depth']),
            float(data['result']['face_x']),
            float(data['result']['face_y']),
            float(data['result']['face_angle']),
            float(data['result']['depth_diff']),
            float(data['result']['left_eye']['angle']),
            float(data['result']['left_eye']['x']),
            float(data['result']['left_eye']['y']),
            float(data['result']['right_eye']['angle']),
            float(data['result']['right_eye']['x']),
            float(data['result']['right_eye']['y'])
        ]]
    )
    data['result']['gaze_prediction']['x'] = prediction[0][0]
    data['result']['gaze_prediction']['y'] = prediction[0][1]
