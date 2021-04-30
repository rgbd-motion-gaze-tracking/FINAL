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

ui = None

##--------------------##
## ROS Topic Handlers ##
##--------------------##

def eye_listener(data):
    print(data)

def face_listener(data):
    pass

def gaze_listener(data):
    pass

class UserInterface(tk.Frame):

    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.pack()
        # Create widgets
        


def listener():
    global ui
    # Ros Stuff
    rospy.Subscriber('tracker_eyes', EyeInfo, eye_listener)
    rospy.Subscriber('tracker_face', FaceInfo, face_listener)
    rospy.Subscriber('tracker_faze', GazeInfo, gaze_listener)
    rospy.init_node('test_node', anonymous=True)
    # Create the user interface
    ui = UserInterface(master=tk.Tk())

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass