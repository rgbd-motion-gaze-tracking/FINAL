#!/usr/bin/python3
'''
Test implementation file.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import trackercore
import util

trackercore.init()

while True:
    try:
        trackercore.loop_once()
    except KeyboardInterrupt:
        util.log("\033[Terminated by user")
        break
