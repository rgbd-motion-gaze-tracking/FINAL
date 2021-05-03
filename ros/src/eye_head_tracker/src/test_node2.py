#!/usr/bin/env python3
'''
ROS Test Node.

This node provides a user interface which provides an overview of readings
from the tracker node. This node visualizes all available information, and
allows the operator to change available configuration values on the fly.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import rospy
import sys
import ansi2html
import os
import re
from PySide6.QtCore import Qt, QFile, Signal
from PySide6.QtWidgets import QWidget, QApplication, QLabel, QPushButton
from PySide6.QtUiTools import QUiLoader
from PySide6.QtGui import QPainter
from eye_head_tracker.msg import EyeInfo, FaceInfo, GazeInfo, LogMessage, Command


ui = {}
terminal = {
    "lines": []
}
publishers = {}

ANSI_CONV = ansi2html.Ansi2HTMLConverter()
DIR = os.path.dirname(os.path.realpath(__file__))

##--------------------------##
## Initialization and Setup ##
##--------------------------##

def main():

    # Initialize QT #

    ui["app"] = QApplication(sys.argv)

    loader = QUiLoader()
    ui["window"] = loader.load(f"{DIR}/ui/mainwindow.ui")
    ui["window"].show()

    ui["elem"] = {}
    def findChildren(e):
        children = e.children()
        for child in children:
            name = child.objectName()
            if name is not None:
                ui["elem"][name] = child
            findChildren(child)
    findChildren(ui["window"])

    # Initialize Widgets
    terminal["css"] = open(f"{DIR}/ui/terminal.css").read()
    log_handler = LogListener()

    gaze_handler = GazeListener()
    face_handler = FaceListener()
    left_eye_handler = EyeWidget("left")
    right_eye_handler = EyeWidget("right")

    # Configuration handlers
    ui["elem"]["config_laserpower"].valueChanged.connect(config_laserpower)

    # Initialize ROS #
    def eye_listener(data):
        left_eye_handler.handle(data.left)
        right_eye_handler.handle(data.right)
    rospy.Subscriber('tracker_eyes', EyeInfo, eye_listener)
    rospy.Subscriber('tracker_face', FaceInfo, face_handler.handle)
    rospy.Subscriber('tracker_gaze', GazeInfo, gaze_handler.handle)
    rospy.Subscriber('tracker_log', LogMessage, log_handler.handle)
    publishers["config"] = rospy.Publisher('tracker_cmd', Command, queue_size=10)
    rospy.init_node('test_node', anonymous=True)

    # Run until killed
    ui["app"].exec_()

##---------------##
## Configuration ##
##---------------##

def config_laserpower(value):
    publishers["config"].publish("set_laserpower", str(value))

##--------------##
## ROS Handlers ##
##--------------##

class EyeWidget(QWidget):
    update_signal = Signal()

    def __init__(self, eye):
        super().__init__()
        ui['elem'][f'group_eyes_{eye}'].layout().addWidget(self)
        self.update_signal.connect(self.repaint)
        self.setFixedWidth(240)  # 16:9 but sill small
        self.setFixedHeight(135) # ^
        self.pos_x = 0.5
        self.pos_y = 0.5

    def paintEvent(self, event):
        painter = QPainter(self)

        width = self.width()
        height = self.height()

        # Draw background
        painter.fillRect(0, 0, width, height, Qt.white)
        # Draw pupil location
        x = self.pos_x * width
        y = self.pos_y * height
        painter.setPen(Qt.black)
        painter.setBrush(Qt.red)
        painter.drawEllipse(x-3,y-3,6,6)
        # Draw frame
        painter.setPen(Qt.gray)
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(0, 0, width-1, height-1)

        painter.end()
        super().paintEvent(event)

    def handle(self, data):
        self.pos_x = data.x
        self.pos_y = data.y
        self.update_signal.emit()


class FaceListener(QWidget):
    update_signal = Signal()

    def __init__(self):
        super().__init__()
        ui['elem']['group_facetarget'].layout().addWidget(self)
        self.update_signal.connect(self.repaint)
        self.facepos = (0.5, 0.5)
        self.facedepth = 500
        self.setFixedWidth(240)  # 16:9 but sill small
        self.setFixedHeight(135) # ^

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(Qt.blue)

        width = self.width()
        height = self.height()

        # Calculate aspect remap factor
        factor = (height * 1.777) / width
        new_width = width * factor
        x_offset = (width / 2) - (new_width / 2)

        # Calculate face position
        x = self.facepos[0] * width * factor + x_offset
        y = self.facepos[1] * height
        # Calculate face circle size
        diameter = 1 - (self.facedepth / 1000)
        diameter = max(min(diameter, 1), 0)
        diameter *= height * 0.7
        # Draw background
        painter.fillRect(x_offset, 0, new_width, height-1, Qt.white)
        # Draw the face circle
        offset = diameter / 2
        painter.setPen(Qt.black)
        painter.setBrush(Qt.red)
        painter.drawEllipse(x - offset, y - offset, diameter, diameter)
        # Draw borders
        painter.setPen(Qt.gray)
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(x_offset, 0, new_width, height-1)

        painter.end()
        super().paintEvent(event)

    def handle(self, data):
        self.facepos = (data.face_x, data.face_y)
        self.facedepth = data.face_depth
        self.update_signal.emit()

class GazeListener(QWidget):
    update_signal = Signal()

    def __init__(self):
        super().__init__()
        ui['elem']['group_gazetarget'].layout().addWidget(self)
        self.update_signal.connect(self.repaint)
        self.targets = [
            (0.5,0.5)
        ]
        self.setFixedWidth(240)  # 16:9 but sill small
        self.setFixedHeight(135) # ^

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(Qt.blue)

        width = self.width()
        height = self.height()

        # Calculate aspect remap factor
        factor = (height * 1.777) / width
        new_width = width * factor
        x_offset = (width / 2) - (new_width / 2)
        # Remap points to the screen space
        adj_points = [(x[0]*width*factor+x_offset,x[1]*height) for x in self.targets]
        # Draw background
        painter.fillRect(x_offset, 0, new_width, height-1, Qt.white)
        # Draw lines
        painter.setPen(Qt.black)
        for i in range(1, len(adj_points)):
            p1 = adj_points[i-1]
            p2 = adj_points[i]
            painter.drawLine(p1[0], p1[1], p2[0], p2[1])
        # Draw points
        painter.setPen(Qt.blue)
        painter.setBrush(Qt.blue)
        for point in adj_points:
            painter.drawEllipse(point[0]-2, point[1]-2, 5, 5)
        # Draw average
        avg_x = sum([x[0] for x in self.targets]) / len(self.targets) * width
        avg_x = avg_x * factor + x_offset
        avg_y = sum([x[1] for x in self.targets]) / len(self.targets) * height
        painter.setPen(Qt.black)
        painter.setBrush(Qt.red)
        painter.drawEllipse(avg_x-6,avg_y-6, 12, 12)
        # Draw Border
        painter.setPen(Qt.gray)
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(x_offset, 0, new_width, height-1)

        painter.end()
        super().paintEvent(event)

    def handle(self, data):
        self.targets.append((data.x, data.y))
        self.targets = self.targets[-10:]
        self.update_signal.emit()


class LogListener(QWidget):
    html_update_signal = Signal(str)
    scroll_update_signal = Signal(int)

    def __init__(self):
        super().__init__()
        self.html_update_signal.connect(ui["elem"]["terminal"].setHtml)
        scroll = ui["elem"]["terminal"].verticalScrollBar()
        self.scroll_update_signal.connect(scroll.setValue)

    def handle(self, data):
        message = ANSI_CONV.convert(data.message, full=False)
        message = re.sub("[\r\n]+$", "", message)
        terminal['lines'].append(message)
        terminal['lines'] = terminal['lines'][-1000:]
        document = f'''
        <html>
            <head>
                <style>{terminal['css']}</style>
            </head>
            <body>
                {'<br>'.join(terminal['lines'])}
            </body>
        </html>
        '''
        self.html_update_signal.emit(document)
        self.scroll_update_signal.emit(int(100000)) # Easier than calculating it


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
