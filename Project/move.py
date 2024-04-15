#!/usr/bin/env python3

# Main Control Loop for the Gerard SLAM project
# Jasper Grant and Michael MacGillivray
# 2024-04-05

from math import pi, sqrt
from time import sleep, time
from ev3dev2.button import Button
from motion_controller import velocity_controller
from detect import is_object_detected

# Motor inputs
from ev3dev2.motor import (
    LargeMotor,
    OUTPUT_A,
    OUTPUT_D,
)

goals_reached = 0

# Load waypoints from file
goals_file = open("goals.csv", "r")
# Set goal lists
x_goal = []
y_goal = []
theta_goal = []
# Load goals into lists
for line in goals_file:
    if line != "":
        x, y, theta = line.split(",")
        x_goal.append(float(x))
        y_goal.append(float(y))
        theta_goal.append(float(theta))

# Set up robot as tank along with sensors
left_motor = LargeMotor(OUTPUT_D)
right_motor = LargeMotor(OUTPUT_A)

# Set up buttons
button = Button()

pose_past = [40, 0, pi / 2]
# thresholds for setpoint tracker
NUM_DEGREES_FOR_EQUALITY = 3 * pi / 180
DISTANCE_FOR_EQUALITY = 5
WRONG_DIRECTION_LIMIT = 10

# Inital test conditons for waypoints. Will be updated to be dynamic
DEPTH_GOAL = 20
WIDTH_GOAL = 100


def turn(left_motor, right_motor, theta_goal):
    global pose_past
    while abs(pose_past[2] - theta_goal) > (NUM_DEGREES_FOR_EQUALITY):
        pose_past = velocity_controller(
            left_motor,
            right_motor,
            pose_past[0],
            pose_past[1],
            theta_goal,
            pose_past,
            is_turning=True,
        )
        if is_object_detected():
            print("Object detected")
            return -2


def move_forward(left_motor, right_motor, x_goal, y_goal, theta_goal):
    global pose_past

    wrong_direction_count = 0
    # print("Moving forward")
    print(pose_past)
    print(x_goal, y_goal, theta_goal)

    while (
        sqrt((x_goal - pose_past[0]) ** 2 + (y_goal - pose_past[1]) ** 2)
        > DISTANCE_FOR_EQUALITY
    ):
        pre_dist = sqrt((x_goal - pose_past[0]) ** 2 + (y_goal - pose_past[1]) ** 2)
        pose_past = velocity_controller(
            left_motor,
            right_motor,
            x_goal,
            y_goal,
            theta_goal,
            pose_past,
            is_turning=False,
        )
        post_dist = sqrt((x_goal - pose_past[0]) ** 2 + (y_goal - pose_past[1]) ** 2)

        if pre_dist < post_dist:
            wrong_direction_count += 1
        if wrong_direction_count > WRONG_DIRECTION_LIMIT:
            print("Goal given up on")
            return -1
        if is_object_detected():
            print("Object detected")
            return -2
    return 0
