#!/usr/bin/python3
'''
Run the Operations UI.

The operations UI is a handy way to launch individual system components.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import sys
import ansi2html
import os
from PySide6.QtCore import Qt, QFile, Signal, Slot
from PySide6.QtWidgets import QWidget, QApplication, QLabel, QPushButton, QTextBrowser
from PySide6.QtUiTools import QUiLoader
from PySide6.QtGui import QPainter
import subprocess
import threading
import re

DIR = os.path.dirname(os.path.realpath(__file__))

##----------------------------##
## Initialization and runtime ##
##----------------------------##

ui = {}

def main():
    # Load the QT UI
    app = QApplication(sys.argv)
    loader = QUiLoader()
    window = loader.load(f"{DIR}/userinterface.ui")
    window.show()
    ui['window'] = window

    widgets = {}
    def findChildren(e):
        children = e.children()
        for child in children:
            name = child.objectName()
            if name is not None:
                widgets[name] = child
            findChildren(child)
    findChildren(window)
    ui['widgets'] = widgets

    terminal = Terminal()
    ui['terminal'] = terminal
    widgets['group_console'].layout().addWidget(terminal)

    widgets['launch_collector'].clicked.connect(lambda: WatchedProcess("./trainer/collector.py", "Collector", widgets['launch_collector']))
    widgets['launch_demo'].clicked.connect(lambda: WatchedProcess("./trainer/demo.py", "Demo", widgets['launch_demo']))
    widgets['launch_tester'].clicked.connect(lambda: WatchedProcess("./trainer/test.py", "Test", widgets['launch_tester']))
    widgets['launch_trainer'].clicked.connect(lambda: WatchedProcess("./trainer/train2.py", "Trainer", widgets['launch_trainers']))
    widgets['launch_ros'].clicked.connect(lambda: WatchedProcess("roslaunch ./ros/src/eye_head_tracker/demo.launch", "ROS", widgets['launch_ros']))

    app.exec_()

##-------------##
## Other Stuff ##
##-------------##

class WatchedProcess(QWidget):

    done_signal = Signal(bool)

    def __init__(self, command, name, widget):
        super().__init__()
        ui['terminal'].write_text(f"\033[3mStarting {name}...\033[0m")
        self.process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.name = name
        self.done_signal.connect(widget.setEnabled)
        self.done_signal.emit(False)
        self.thread1 = threading.Thread(target=self.__stdoutthread)
        self.thread2 = threading.Thread(target=self.__stderrthread)
        self.thread1.start()
        self.thread2.start()

    def __stdoutthread(self):
        while True:
            out = self.process.stdout.readline().decode()
            if out == '' and self.process.poll() != None:
                break
            if out != '':
                ui['terminal'].write_text(f"[{self.name}] {out}")
        self.done_signal.emit(True)
        ui['terminal'].write_text(f"\033[3m{self.name} complete.\033[0m")

    def __stderrthread(self):
        while True:
            out = self.process.stderr.readline().decode()
            if out == '' and self.process.poll() != None:
                break
            if out != '':
                ui['terminal'].write_text(f"\033[31m[{self.name}] {out}")

class Terminal(QTextBrowser):
    __write_signal = Signal(str)

    def __init__(self):
        super().__init__()
        self.__write_signal.connect(self.__write)
        self.css = open(f"{DIR}/ros/src/eye_head_tracker/src/ui/terminal.css").read()
        self.lines = []
        self.max_lines = 1000
        self.ansi_converter = ansi2html.Ansi2HTMLConverter()
        self.setObjectName('terminal')
        self.setStyleSheet('''
        #terminal {
            background-color: black;
            color: white;
            font: monospace;
        }
        ''')
    
    def write_text(self, message):
        self.__write_signal.emit(message)

    @Slot()
    def __write(self, message):
        message = re.sub("[\r\n]+$", "", message)
        message = self.ansi_converter.convert(message, full=False)
        self.lines.append(message)
        self.lines = self.lines[-self.max_lines:]
        html = f'''
        <html>
            <head><style>{self.css}</style></head>
            <body>{'<br>'.join(self.lines)}</body>
        </html>'''
        self.setHtml(html)
        self.verticalScrollBar().setValue(100000) # Easier than calculating

if __name__ == '__main__':
    main()
