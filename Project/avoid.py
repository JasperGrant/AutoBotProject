#!/usr/bin/env python3

# Obstacle Avoidance functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-13

from math import cos, sin, radians, degrees, pi

# Sensor inputs
from ev3dev2.sensor import INPUT_1

# Sensor types
from ev3dev2.sensor.lego import UltrasonicSensor

from ev3dev2.motor import (
    MediumMotor,
    OUTPUT_B,
)

avoidance_servo = MediumMotor(OUTPUT_B)
avoidance_ultrasonic_sensor = UltrasonicSensor(address=INPUT_1)

OBJECT_DETECTION_DISTANCE = 10


def is_object_detected():
    return avoidance_ultrasonic_sensor.distance_centimeters < OBJECT_DETECTION_DISTANCE
