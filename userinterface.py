#!/usr/bin/python3
'''
Run the Operations UI.

The operations UI is a handy way to launch individual system components.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import tkinter as tk
import subprocess

config = {
    "external_console": False
}

elements = {
}

def launch_demo():
    if config["external_console"]:
        command = f'terminator --title "TrackerCore Demonstration - Press Q to exit" --command "./trainer/demo.py"'
        subprocess.Popen(command, shell=True)
    else:
        command = f'./trainer/demo.py'
        subprocess.Popen(command, shell=True)

def external_console_clicked():
    config['external_console'] = elements['external_console_var'].get() > 0

def main():
    root = tk.Tk()
    root.title("Operations UI")

    button_frame = tk.Frame(root)
    button_frame.pack()

    button_row1 = tk.Frame(button_frame)
    button_row1.pack(side = tk.TOP)

    button_row2 = tk.Frame(button_frame)
    button_row2.pack(side = tk.BOTTOM)

    button_demo = tk.Button(button_row1, text="Launch Demo", command=launch_demo)
    button_demo.pack(side = tk.LEFT)

    button_train = tk.Button(button_row1, text="Train Model")
    button_train.pack(side = tk.RIGHT)

    button_collect = tk.Button(button_row2, text="Collect Training Data")
    button_collect.pack(side = tk.LEFT)

    button_testmodel = tk.Button(button_row2, text="Test Model")
    button_testmodel.pack(side = tk.RIGHT)

    external_console_var = tk.IntVar()
    external_console_check = tk.Checkbutton(root, text="Use External Console", command=external_console_clicked, variable=external_console_var)
    external_console_check.pack()

    external_console = 

    elements['button_frame'] = button_frame
    elements['button_row1'] = button_row1
    elements['button_row2'] = button_row2
    elements['button_demo'] = button_demo
    elements['button_train'] = button_train
    elements['button_collect'] = button_collect
    elements['button_testmodel'] = button_testmodel
    elements['external_console_check'] = external_console_check
    elements['external_console_var'] = external_console_var

    root.mainloop()

if __name__ == '__main__':
    main()