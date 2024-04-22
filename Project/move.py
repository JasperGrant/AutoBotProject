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
    goal_directions = []
    x_goal = []
    y_goal = []
    theta_goal = []
    # Load goals into lists
    for line in goals_file:
        if line != "":
            direction, x, y, theta = line.split(",")
            goal_directions.append(direction)
            x_goal.append(float(x))
            y_goal.append(float(y))
            theta_goal.append(float(theta))
    return goal_directions, x_goal, y_goal, theta_goal


goal_directions, x_goal, y_goal, theta_goal = get_goals()


def get_goal_directions():
    return goal_directions


def get_x_goal():
    return x_goal


def get_y_goal():
    return y_goal


def get_theta_goal():
    return theta_goal


# Set up buttons
button = Button()

# thresholds for setpoint tracker
RADS_FOR_EQUALITY = 5 * pi / 180
DISTANCE_FOR_EQUALITY = 5
WRONG_DIRECTION_LIMIT = 5


def turn(left_motor, right_motor, theta_goal):
    pose_past = get_pose_past()

    while abs(get_pose_past()[2] - theta_goal) > (RADS_FOR_EQUALITY):
        print(get_pose_past())
        velocity_controller(
            left_motor,
            right_motor,
            pose_past[0],
            pose_past[1],
            theta_goal,
            is_turning=True,
        )


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
