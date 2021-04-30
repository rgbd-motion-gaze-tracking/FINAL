#!/usr/bin/python3
'''
Trainer Test.

This model tests the tensorflow model and generates an accuracy heatmap.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import numpy as np
import cv2

from core import trackercore, util, debug

RESOLUTION_X = 1920
RESOLUTION_Y = 1080

data = {}

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

    trackercore.init()

    # Create the application window
    data['window_name'] = "Tracker"
    cv2.namedWindow(data['window_name'], cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(data['window_name'], cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    # Create the output window
    data['window'] = np.zeros((RESOLUTION_Y, RESOLUTION_X, 3), np.uint8)

def run():
    '''
    Main run loop. This is where the data gathering happens.
    '''
    last = []
    while True:
        # Take a reading
        result = trackercore.loop_once()

        # Clear the window
        cv2.rectangle(data['window'], (0, 0), (RESOLUTION_X, RESOLUTION_Y), (255, 255, 255), -1)

        prediction = result['gaze_prediction']

        predict_x = int(prediction['x'] * RESOLUTION_X)
        predict_y = int(prediction['y'] * RESOLUTION_Y)

        last.append((predict_x, predict_y))
        last = last[-10:]

        cv2.circle(data['window'], (predict_x, predict_y), 10, (255, 0, 0), -1)
        for point in last:
            cv2.circle(data['window'], point, 5, (0, 0, 255), -1)
        average_x = int(sum([x[0] for x in last]) / len(last))
        average_y = int(sum([x[1] for x in last]) / len(last))
        
        cv2.circle(data['window'], (average_x, average_y), 10, (0, 255, 0), -1)

        # Show the data
        cv2.imshow(data['window_name'], data['window'])
        # Check for key
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            return

init()
run()