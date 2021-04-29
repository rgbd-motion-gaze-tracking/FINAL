#!/usr/bin/python3
'''
Evaluates the accuracy of the tensorflow model.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import tensorflow as tf
import numpy as np
import cv2

import math

print("Loading data")
labels = []
readings = []
data = open("data.csv", "r")
for line in data:
    parts = line.split(",")
    if len(parts) == 1:
        continue
    labels.append([
        float(parts[0]),
        float(parts[1])
    ])
    readings.append([float(x) for x in parts[2:]])

print("Loading model")
model = tf.keras.models.load_model("model")

print("Testing data")
predictions = model.predict(readings)

RESOLUTION_X = 1920*2
RESOLUTION_Y = 1080*2
FILL_RADIUS = 50
final_map = np.zeros((RESOLUTION_Y, RESOLUTION_X, 2), dtype=np.float)

# Initialize the final map
print("Preparing result map")
for y in range(0, RESOLUTION_Y):
    for x in range(0, RESOLUTION_X):
        final_map[y,x,1] = math.inf

print("Processing results")
diffsum = 0
diffcount = 0
for i, result in enumerate(predictions):
    print(f"\r{i}/{len(predictions)}", end="")
    truth = labels[i]
    dx = result[0] - truth[0]
    dy = result[1] - truth[1]
    diff = math.sqrt(dx*dx + dy*dy)

    diffsum += diff
    diffcount += 1

    fx = int(truth[0] * RESOLUTION_X)
    fy = int(truth[1] * RESOLUTION_Y)
    
    for y in range(-FILL_RADIUS, FILL_RADIUS):
        for x in range(-FILL_RADIUS, FILL_RADIUS):
            if 0 <= fx + x < RESOLUTION_X:
                if 0 <= fy + y < RESOLUTION_Y:
                    dist = math.sqrt(x*x + y*y)
                    ox = fx + x
                    oy = fy + y
                    if dist < final_map[oy, ox, 1]:
                        final_map[oy, ox, 1] = dist
                        final_map[oy, ox, 0] = diff
print()

# Generate final image data
print("Generating final image")
final_image = np.zeros((RESOLUTION_Y, RESOLUTION_X), dtype=np.uint8)
for y in range(0, RESOLUTION_Y):
    for x in range(0, RESOLUTION_X):
        final_image[y, x] = final_map[y, x, 0] * 255

img = cv2.applyColorMap(final_image, cv2.COLORMAP_JET)
cv2.imwrite("test2.png", img)

print(f"The average error is {diffsum/diffcount}")
