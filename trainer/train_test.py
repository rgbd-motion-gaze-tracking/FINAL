#!/usr/bin/python3
'''
Trainer Test.

This model tests the tensorflow module and generates an accuracy heatmap.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import math

import tensorflow as tf
import numpy as np
import cv2

RESOLUTION_X = 1920
RESOLUTION_Y = 1080

print("Loading model")
model = tf.keras.models.load_model("model")

result = np.zeros((RESOLUTION_X, RESOLUTION_Y, 2), dtype=np.float)

print("Testing")
data = open("data.csv")
diffsum = 0
diffcount = 0
for i, line in enumerate(data):
    # Parse line
    parts = line.split(",")
    if len(parts) == 1:
        continue
    # Get values
    x = float(parts[0])
    y = float(parts[1])
    inp = [float(x) for x in parts[2:]]
    # Execute prediction
    foo = model.predict([inp])
    # Calculate error
    dx = x - foo[0][0]
    dy = y - foo[0][1]
    diff = math.sqrt(dx**2 + dy**2)
    # Log difference
    rx = int(RESOLUTION_X * x)
    ry = int(RESOLUTION_Y * y)
    result[rx, ry, 0] += diff
    result[rx, ry, 1] += 1
    diffsum += diff
    diffcount += 1
    #if i > 1000:
    #    break
    print(i)

print(f"\033[1mAverage diff is {diffsum/diffcount}.")

print("Remapping")
final = np.zeros((RESOLUTION_Y, RESOLUTION_X), dtype=np.uint8)
vpix = 0
for y in range(0, RESOLUTION_Y):
    for x in range(0, RESOLUTION_X):
        if result[x, y, 1] > 0:
            vpix += 1
            final[y, x] = result[x, y, 0] / result[x, y, 1] * 255
        else:
            final[y, x] = 0
print(f"There were {vpix} valid pixels")

img = cv2.applyColorMap(final, cv2.COLORMAP_JET)
cv2.imwrite("test.png", img)
