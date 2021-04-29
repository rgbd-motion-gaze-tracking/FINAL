#!/usr/bin/python3
'''
Trainer Module.

This model trains the tensorflow model.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import os
import random

import tensorflow as tf
import numpy as np

# Comment this line out of you're not cool and bought an nvidia gpu
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

BATCH_SIZE = 64

# Load the data
print("Loading Data")
labels = []
examples = []
data = open("data.csv","r")
scrambled_data = data.read().split("\n")
random.shuffle(scrambled_data)
for line in scrambled_data:
    parts = line.split(",")
    if len(parts) == 1:
        continue
    labels.append([
        float(parts[0]),
        float(parts[1])
    ])
    examples.append([float(x) for x in parts[2:]])
del scrambled_data # Clear up memory

# Separate the data into training and testing sets
print("Separating Data")
separation = int(len(examples) * 0.8)
train_examples = np.asarray(examples[:separation], dtype=np.float32)
#train_examples = np.transpose(train_examples)
train_labels = np.asarray(labels[:separation], dtype=np.float32)
#train_labels = np.transpose(train_labels)
test_examples = np.asarray(examples[separation:], dtype=np.float32)
#test_examples = np.transpose(test_examples)
test_labels = np.asarray(labels[separation:], dtype=np.float32)
#test_labels = np.transpose(test_labels)

print(train_examples.shape)
print(train_labels.shape)
print(test_examples.shape)
print(test_labels.shape)


# Create tensorflow datasets
print("Creating Datasets")
train_dataset = tf.data.Dataset.from_tensor_slices((train_examples, train_labels))
train_batch = train_dataset.batch(BATCH_SIZE)
test_dataset = tf.data.Dataset.from_tensor_slices((test_examples, test_labels))
test_batch = test_dataset.batch(BATCH_SIZE)

# Create the model
print("Creating Model")
def custom_loss(true, pred):
    '''
    Custom loss function for computing error based on square route distance from target.
    '''
    return tf.keras.backend.sqrt(tf.keras.backend.sum(tf.keras.backend.square(pred - true), axis=-1))

model = tf.keras.Sequential([
    tf.keras.layers.InputLayer(input_shape=(11, 1)),
    tf.keras.layers.Dense(256),
    tf.keras.layers.Dense(512),
    tf.keras.layers.Dense(2)
])
model.compile(optimizer=tf.keras.optimizers.RMSprop(),
    loss=custom_loss,
    metrics=[])

print("Summary:")
print(model.summary())

# Train the model
print("Training model")
model.fit(train_dataset, epochs=1000)

# Test the model
print("Testing model")
model.evaluate(test_dataset)

print("Saving model")
model.save("model")
