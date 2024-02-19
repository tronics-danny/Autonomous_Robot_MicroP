# Autonomous Robot Project

## Overview

This project focuses on designing an autonomous robot capable of navigating a predefined track while avoiding obstacles. The robot will utilize various sensors and algorithms to achieve its objectives.

## Objectives

- **Track Following**: Develop algorithms to enable the robot to follow a specified track accurately.
- **Obstacle Avoidance**: Implement obstacle detection and avoidance mechanisms to ensure the robot can navigate around obstacles effectively.
- **Autonomous Operation**: Enable the robot to operate independently without human intervention once activated.
- **Collaborative Development**: Host the project on GitHub to facilitate collaboration among team members and allow for efficient development and version control.

## Operation Summary

1. **Initialization**: The robot is powered on and initializes its sensors and systems.
2. **Track Detection**: Using IR sensors, the robot identifies the track and determines its path.
3. **Navigation**: The robot follows the track while continuously scanning for obstacles in its path.
4. **Obstacle Detection**: Upon detecting an obstacle, the robot adjusts its path to avoid collision.
5. **Continuous Operation**: The robot repeats the navigation process autonomously until it reaches the destignated target area or manually stopped or encountering an issue.

## Components

### Sensors
- **Ultrasonic Sensor**: Used for detecting obstacles in the robot's path and measuring distances.
- **IR Sensors**: Utilized for line following to keep the robot on track.

### Other Components
- **DC Motors (x2)**: Drive the robot's movement along the track.
- **Motor Driver**: Controls the speed and direction of the DC motors.
- **Servo Motor**: Mounts the ultrasonic sensor, enabling it to scan the environment within a specified range.
