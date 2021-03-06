#!/usr/bin/python3
'''
Test implementation file.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import trackercore

trackercore.init()

while True:
    trackercore.loop_once()
