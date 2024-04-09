#!/usr/bin/env python3

# Sensor functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-06

from time import sleep
from math import cos, sin, radians
import random

# Sensor inputs
from ev3dev2.sensor import INPUT_3

# Sensor types
from ev3dev2.sensor.lego import UltrasonicSensor

from ev3dev2.motor import (
    MediumMotor,
    OUTPUT_C,
)

servo = MediumMotor(OUTPUT_C)
ultrasonic_sensor = UltrasonicSensor(address=INPUT_3)


def reset_servo():
    servo.reset()


def move_servo_to_angle(angle, speed=10, callback=lambda: sleep(0.1)):
    servo.on_to_position(speed, angle)
    while servo.is_running:
        callback()


def get_ultrasonic_distance():
    return ultrasonic_sensor.distance_centimeters


def transform_distance_to_coordinate(distance, angle, robot_pose):
    return [
        robot_pose[0] + (distance * (cos(radians(angle) + robot_pose[2]))),
        robot_pose[1] + (distance * (sin(radians(angle) + robot_pose[2]))),
    ]


def sensor_scan(width, resolution, robot_pose):
    points = []
    for angle in range(0, -width, -resolution):
        move_servo_to_angle(angle)
        distance = get_ultrasonic_distance()
        if distance == 255:
            continue
        map_file = open("map.txt", "a")
        point = transform_distance_to_coordinate(distance, -angle, robot_pose)
        points.append(point)
        map_file.write(str(point[0]) + " , " + str(point[1]) + "\n")
        map_file.close()
        sleep(0.2)
    return points


# Test the sensor scan function
if __name__ == "__main__":
    servo.reset()
    # Clear map file
    map_file = open("map.txt", "w")
    map_file.write("")
    map_file.close()
    print("Starting scan")
    sensor_scan(360, 5, (60.96, 122.88, 0))
    print("Scan complete")
