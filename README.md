# Robotic Teleoperation via Motion and Gaze Tracking

This repository contains code and documentation for the Robotic Teleoperation
via Motion and Gaze Tracking MQP, completed by Nicholas Hollander during the
2020-2021 Academic Year.

## Project Overview

This project is broken down into three main components. `core` contains all of
the major logic components of the project. If something is broken, it's probably
broken in here. `rosnode` provides a wrapper around `core` that is accessible as
a ROS node. `ui` provides a user interface for the core module.

Documentation for each component can be found at the following links:
 - [Core Documentation](./core/README.md)
 - [ROS Node Documentation](./rosnode/README.md)
 - [UI Documentation](./ui/README.md)

## Requirements

In order to run this tracking system, you need to meet a few basic requirements,
outlined below:

| Requirement          | Description / Justification
|----------------------|-----------------------------------------------|
| Intel Realsense D435 | This tracking system is built around the Intel RealSense RGB+D camera system. It should be relatively easy to modify the code to support other models and brands of depth camera, but this is all that I was able to test on. |
| Python | Required python libraries can be instealled with `python3 -m pip install -r requirements.txt`. More information on each requirement can be found in thiat file. |
| Linux 🐧 | In theory it should run on other operating systems, but it's completely untested, and will probably explode or something. Who knows.

Additional requirements for `core` and `rosnode` can be found in their
respective READMEs.

## Running

You can launch the standalone user interface by running `./runui.sh`