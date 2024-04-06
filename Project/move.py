#!/usr/bin/env python3

# Main Control Loop for the Gerard SLAM project
# Jasper Grant and Michael MacGillivray
# 2024-04-05

import math as m
from time import sleep, time

from motion_controller import velocity_controller

# Motor inputs
from ev3dev2.motor import (
    MoveTank,
    LargeMotor,
    OUTPUT_A,
    OUTPUT_D,
)

# # Sensor inputs
# from ev3dev2.sensor import INPUT_1, INPUT_4

# # Sensor types
# from ev3dev2.sensor.lego import ColorSensor

# Button
from ev3dev2.button import Button

# Set up robot as tank along with sensors
robot = MoveTank(OUTPUT_D, OUTPUT_A)
left_motor = LargeMotor(OUTPUT_D)
right_motor = LargeMotor(OUTPUT_A)

# Set up buttons
button = Button()

pose_past = [0, 0, 0]

PI = 3.14159

# thresholds for setpoint tracker
NUM_DEGREES_FOR_EQUALITY = 3 * PI / 180
DISTANCE_FOR_EQUALITY = 10
WRONG_DIRECTION_LIMIT = 10

# Inital test conditons for waypoints. Will be updated to be dynamic
DEPTH_GOAL = 100
WIDTH_GOAL = 100


def move_robot():
    # Move in a square pattern
    i = 0
    while i < 1:
        print("segment", i)
        x_goal = [DEPTH_GOAL, DEPTH_GOAL, 0]

        y_goal = [0, WIDTH_GOAL, WIDTH_GOAL]
        theta_goal = [0, PI / 2, PI]

        turn(left_motor, right_motor, theta_goal[i])
        move_forward(left_motor, right_motor, x_goal[i], y_goal[i], theta_goal[i])

        i += 1


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


def move_forward(left_motor, right_motor, x_goal, y_goal, theta_goal):
    global pose_past

    wrong_direction_count = 0

    while (
        m.sqrt((x_goal - pose_past[0]) ** 2 + (y_goal - pose_past[1]) ** 2)
        > DISTANCE_FOR_EQUALITY
    ):
        pre_dist = m.sqrt((x_goal - pose_past[0]) ** 2 + (y_goal - pose_past[1]) ** 2)
        pose_past = velocity_controller(
            left_motor,
            right_motor,
            x_goal,
            y_goal,
            theta_goal,
            pose_past,
            is_turning=False,
        )
        post_dist = m.sqrt((x_goal - pose_past[0]) ** 2 + (y_goal - pose_past[1]) ** 2)

        print("pose", pose_past)

        if pre_dist < post_dist:
            wrong_direction_count += 1
        if wrong_direction_count > WRONG_DIRECTION_LIMIT:
            print("Goal given up on")
            return False


def main():
    while not button.any():
        if not move_robot():
            break


if __name__ == "__main__":
    main()
