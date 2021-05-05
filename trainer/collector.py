#!/usr/bin/python3
'''
Training data collection module.

This module collects eye data which can be used to generate the eye tracking
model.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import math
import random

import cv2
import numpy as np

from core import trackercore, util, debug

RESOLUTION_X = 1920
RESOLUTION_Y = 1080
RES_XY = (RESOLUTION_X, RESOLUTION_Y)

KEY_PAUSE = " "
KEY_DELETE = "d"
KEY_SAVE = "q"

INSTRUCTIONS = f'''
Pause and Resume by pressing the [{util.human_name(KEY_PAUSE)}] key.
Press [{util.human_name(KEY_DELETE)}] to delete the last 20 readings.
This program doesn't filter out blinking!
If you blink, quickly press [{util.human_name(KEY_DELETE)}] to prevent data corruption.
To Start:
 1. Move mouse over this window
 2. Resume ([{util.human_name(KEY_PAUSE)}])
 3. Move the mouse all over the screen
 4. Follow mouse with your eyes
 5. Gently move your head around, changing angles and distance slightly.
 6. Once you're finished, stop and save by pressing [{util.human_name(KEY_SAVE)}].
'''

data = {
    "paused": True,
    "mouse_x": 0,
    "mouse_y": 0,
    "readings": []
}

@debug.funcperf
def update_mouse(_, x, y, __, ___):
    '''
    Mouse update event handler.
    '''
    data['mouse_x'] = x / RESOLUTION_X
    data['mouse_y'] = y / RESOLUTION_Y

@debug.funcperf
def init():
    '''
    Initialize the training data collection system.
    '''

    util.log("Initializing data collector")

    # Disable all debug settings to increase speed and prevent windows from
    # appearing
    for dbg in trackercore.config['debug']:
        if trackercore.config['debug'][dbg] is True:
            trackercore.config['debug'][dbg] = False
    trackercore.config['enable_prediction'] = False

    trackercore.init()

    # Create the application window
    data['window_name'] = "Tracker"
    cv2.namedWindow(data['window_name'], cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(data['window_name'], cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.setMouseCallback(data['window_name'], update_mouse)

    # Create the output window
    data['window'] = np.zeros((RESOLUTION_Y, RESOLUTION_X, 3), np.uint8)
    data['blank_img'] = np.zeros((RESOLUTION_Y, RESOLUTION_X, 3), np.uint8)

    # Create the pause window
    data['img_paused'] = np.zeros((RESOLUTION_Y, RESOLUTION_X, 3), np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.rectangle(data['img_paused'], (0, 0), RES_XY, (255, 255, 255), -1)
    cv2.putText(data['img_paused'], "PAUSED", (10, 100), font, 4, (20, 180, 0), 4, cv2.LINE_AA)
    cv2.putText(data['img_paused'], "Instructions:", (50, 175), font, 1, (0, 0, 0), 2, cv2.LINE_AA)
    for i, line in enumerate(INSTRUCTIONS.split('\n')):
        y = 180 + (i * 30)
        cv2.putText(data['img_paused'], line, (50, y), font, 1, (0, 0, 0), 1, cv2.LINE_AA)

@debug.funcperf
def collect():
    '''
    Main collection loop. This is where the data gathering happens.
    '''
    while True:
        # Take a reading
        trackercore.loop_once()

        if data['paused']:
            # Copy the pause window with instructions
            data['window'][:, :, :] = data['img_paused']
            # Show the current readings
            readings = "Raw Readings:\n" + \
            f"mouse X:         {data['mouse_x']}\n" + \
            f"mouse Y:         {data['mouse_y']}\n" + \
            f"valid:           {trackercore.data['result']['valid']}\n" + \
            f"face_depth:      {trackercore.data['result']['face_depth']}\n" + \
            f"face_x:          {trackercore.data['result']['face_x']}\n" + \
            f"face_y:          {trackercore.data['result']['face_y']}\n" + \
            f"left_eye.angle:  {trackercore.data['result']['left_eye']['angle']}\n" + \
            f"left_eye.x:      {trackercore.data['result']['left_eye']['x']}\n" + \
            f"left_eye.y:      {trackercore.data['result']['left_eye']['y']}\n" + \
            f"right_eye.angle: {trackercore.data['result']['right_eye']['angle']}\n" + \
            f"right_eye.x:     {trackercore.data['result']['right_eye']['x']}\n" + \
            f"right_eye.y:     {trackercore.data['result']['right_eye']['y']}\n" + \
            f"face_angle:      {trackercore.data['result']['face_angle']}\n" + \
            f"depth_diff:      {trackercore.data['result']['depth_diff']}\n" + \
            f"reading_count:   {len(data['readings'])}"
            for i, line in enumerate(readings.split('\n')):
                y = 600 + (i * 30)
                cv2.putText(
                    data['window'], line, (50, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1, cv2.LINE_AA)

        else:
            # Clear the window
            data['window'][:, :, :] = data['blank_img']
            # Record
            if trackercore.data['result']['valid']:
                data['readings'].append({
                    "mouse_x": data['mouse_x'],
                    "mouse_y": data['mouse_y'],
                    "face_depth": trackercore.data['result']['face_depth'],
                    "face_x": trackercore.data['result']['face_x'],
                    "face_y": trackercore.data['result']['face_y'],
                    "face_angle": trackercore.data['result']['face_angle'],
                    "depth_diff": trackercore.data['result']['depth_diff'],
                    "left_eye_angle": trackercore.data['result']['left_eye']['angle'],
                    "left_eye_x": trackercore.data['result']['left_eye']['x'],
                    "left_eye_y": trackercore.data['result']['left_eye']['y'],
                    "right_eye_angle": trackercore.data['result']['right_eye']['angle'],
                    "right_eye_x": trackercore.data['result']['right_eye']['x'],
                    "right_eye_y": trackercore.data['result']['right_eye']['y']
                })
        # Show the data
        cv2.imshow(data['window_name'], data['window'])
        # Check for key
        key = cv2.waitKey(1) & 0xFF
        if key == ord(KEY_PAUSE):
            data['paused'] = not data['paused']
        if key == ord(KEY_DELETE):
            data['readings'] = data['readings'][:-20]
        if key == ord(KEY_SAVE):
            return

@debug.funcperf
def save():
    '''
    Write the collected values to a CSV file for later processing
    '''
    util.log(f"There are {len(data['readings'])} readings!")
    out = open("data.csv", "a")
    for line in data['readings']:
        out.write(f"{line['mouse_x']},")
        out.write(f"{line['mouse_y']},")
        out.write(f"{line['face_depth']},")
        out.write(f"{line['face_x']},")
        out.write(f"{line['face_y']},")
        out.write(f"{line['face_angle']},")
        out.write(f"{line['depth_diff']},")
        out.write(f"{line['left_eye_angle']},")
        out.write(f"{line['left_eye_x']},")
        out.write(f"{line['left_eye_x']},")
        out.write(f"{line['right_eye_angle']},")
        out.write(f"{line['right_eye_x']},")
        out.write(f"{line['right_eye_x']}\n")
    out.flush()
    out.close()
    util.log("Write complete!")

init()
collect()
save()