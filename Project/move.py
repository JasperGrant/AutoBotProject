#!/usr/bin/env python3

# Main Control Loop for the Gerard SLAM project
# Jasper Grant and Michael MacGillivray
# 2024-04-05

from math import pi, sqrt
from ev3dev2.button import Button
from motion_controller import velocity_controller
from detect import get_avoidance_in_progress
from odometry import get_pose_past


def get_goals():
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
    return x_goal, y_goal, theta_goal


x_goal, y_goal, theta_goal = get_goals()


def get_x_goal():
    return x_goal


def get_y_goal():
    return y_goal


# Set up buttons
button = Button()

# thresholds for setpoint tracker
NUM_DEGREES_FOR_EQUALITY = 3 * pi / 180
DISTANCE_FOR_EQUALITY = 5
WRONG_DIRECTION_LIMIT = 5

# Inital test conditons for waypoints. Will be updated to be dynamic
DEPTH_GOAL = 20
WIDTH_GOAL = 100


def turn(left_motor, right_motor, theta_goal):
    pose_past = get_pose_past()

    while abs(get_pose_past()[2] - theta_goal) > (NUM_DEGREES_FOR_EQUALITY):
        velocity_controller(
            left_motor,
            right_motor,
            pose_past[0],
            pose_past[1],
            theta_goal,
            is_turning=True,
        )
        if get_avoidance_in_progress():
            print("Object detected")
            return -2


def move_forward(left_motor, right_motor, x_goal, y_goal, theta_goal):
    wrong_direction_count = 0
    while (
        sqrt((x_goal - get_pose_past()[0]) ** 2 + (y_goal - get_pose_past()[1]) ** 2)
        > DISTANCE_FOR_EQUALITY
    ):

        pre_dist = sqrt(
            (x_goal - get_pose_past()[0]) ** 2 + (y_goal - get_pose_past()[1]) ** 2
        )
        velocity_controller(
            left_motor,
            right_motor,
            x_goal,
            y_goal,
            theta_goal,
            is_turning=False,
        )
        post_dist = sqrt(
            (x_goal - get_pose_past()[0]) ** 2 + (y_goal - get_pose_past()[1]) ** 2
        )

        if pre_dist < post_dist:
            wrong_direction_count += 1
        if wrong_direction_count > WRONG_DIRECTION_LIMIT:
            print("Goal given up on")
            return -1
        if get_avoidance_in_progress():
            print("Object detected")
            return -2
    return 0
