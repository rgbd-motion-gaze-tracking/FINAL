#!/usr/bin/python3
'''
Test implementation file.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

from core import trackercore
from core import util

trackercore.init()

while True:
    try:
        trackercore.loop_once()
    except KeyboardInterrupt:
        util.log("\033[31mTerminated by user")
        break
