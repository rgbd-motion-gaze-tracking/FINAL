#!/usr/bin/env python3
'''
Teleoperation Tracker Node.

This node must be run on the same computer to which the Intel RealSense camera is connected.

This model tests the tensorflow model and generates an accuracy heatmap.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import tkinter as tk
import rospy
from eye_head_tracker.msg import EyeInfo, FaceInfo, GazeInfo
import time

data = {
    'eye': EyeInfo(),
    'face': FaceInfo(),
    'gaze': GazeInfo()
}

def listener():
    # Ros Stuff
    rospy.Subscriber('tracker_eyes', EyeInfo, eye_listener)
    rospy.Subscriber('tracker_face', FaceInfo, face_listener)
    rospy.Subscriber('tracker_gaze', GazeInfo, gaze_listener)
    rospy.init_node('test_node', anonymous=True)
    # Run until keyboard exception or crash
    rospy.spin()

##--------------------##
## ROS Topic Handlers ##
##--------------------##

def eye_listener(data_):
    data['eye'] = data_
    print_ui()

def face_listener(data_):
    data['face'] = data_

def gaze_listener(data_):
    data['gaze'] = data_

##----------------##
## User Interface ##
##----------------##

def print_ui():
    print("\033[97;44m\033[2J\033[0;0H",end="") # Set appearance and clear screen
    print("\033[1;97;100m\033[0K \U0001F916 TrackerCore ROS Test Node\033[22m")
    print("\033[0;97;44m",end="")
    # Print readings
    print("\033[1mEyes:\033[22m")
    print(data['eye'])
    print("\033[1mFace:\033[22m")
    print(data['face'])
    print("\033[1mGaze:\033[22m")
    print(data['gaze'])


##-------------##
## Other Stuff ##
##-------------##

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("KBE")
        pass
    finally:
        # Clear it out
        print("\033[033;0m\033[2J\033[0;0H",end="")