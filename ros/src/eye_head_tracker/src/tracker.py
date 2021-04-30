#!/usr/bin/env python3
'''
Teleoperation Tracker Node.

This node must be run on the same computer to which the Intel RealSense camera is connected.

This model tests the tensorflow model and generates an accuracy heatmap.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import rospy
import os
import sys
from eye_head_tracker.msg import EyeInfo, EyePoint, FaceInfo, GazeInfo, Command

# I don't know how to make the import work without this line :(
sys.path.append('/home/nicholas/Documents/wpi_files/MQP/THIS_IS_THE_FINAL_TRACKER')
from core import trackercore

def command_handler(command):
    '''
    Tracker module command handler. Allows tracking parameters to be adjusted
    by other ROS nodes.

    Tracking parameters are sent over the `tracker_cmd` ROS topic.

    When a command is received, this node will attempt to execute a
    corresponding command function in the ROS core using the
    `command_{command}(value)` signature.
    '''
    # Check if the command has a handler
    method = getattr(trackercore, f"command_{command.command}", None)
    if method is None:
        rospy.logerr(f"Invalid command received: no handler for [{command.command}]")
        return
    rospy.logdebug(f"Command: {command.command} : {command.value}")
    method(command.value)

def tracker():
    # ROS Stuff
    pub_eye = rospy.Publisher('tracker_eyes', EyeInfo, queue_size=10)
    pub_face = rospy.Publisher('tracker_face', FaceInfo, queue_size=10)
    pub_gaze = rospy.Publisher('tracker_gaze', GazeInfo, queue_size=10)
    rospy.Subscriber('tracker_cmd', Command, command_handler)
    rospy.init_node('tracker', anonymous=True)
    # Disable all debug options that I accidentally left enabled
    for dbg in trackercore.config['debug']:
        if trackercore.config['debug'][dbg] is True:
            trackercore.config['debug'][dbg] = False
    trackercore.init()
    # Track until the heatdeath of the universe and/or the program is terminated
    while not rospy.is_shutdown():
        result = trackercore.loop_once()
        if not result['valid']:
            continue
        eye_left = EyePoint(
            result['left_eye']['angle'],
            result['left_eye']['x'],
            result['left_eye']['y']
        )
        eye_right = EyePoint(
            result['right_eye']['angle'],
            result['right_eye']['x'],
            result['right_eye']['y']
        )
        eye_info = EyeInfo(eye_left, eye_right)
        face_info = FaceInfo(
            result['face_depth'],
            result['face_x'],
            result['face_y'],
            result['face_angle'],
            result['depth_diff']
        )
        gaze_info = GazeInfo(
            result['gaze_prediction']['x'],
            result['gaze_prediction']['y']
        )
        pub_eye.publish(eye_info)
        pub_face.publish(face_info)
        pub_gaze.publish(gaze_info)

if __name__ == '__main__':
    try:
        tracker()
    except rospy.ROSInterruptException:
        pass