#!/usr/bin/env python3

# Movement functions for the Gerard SLAM project
# Jasper Grant and Michael MacGillivray
# 2024-04-05

from math import pi, sqrt
from time import time
from ev3dev2.button import Button
from motion_controller import velocity_controller
from detect import get_avoidance_in_progress
from odometry import get_pose_past
from goals import get_goals_reached, check_give_up_on_goal

TIME_TO_RECALCULATE_PRIORITIES = 10

# Set up buttons
button = Button()

# thresholds for setpoint tracker
RADS_FOR_EQUALITY = 5 * pi / 180
DISTANCE_FOR_EQUALITY = 5
WRONG_DIRECTION_LIMIT = 5


def turn(left_motor, right_motor, theta_goal):
    pose_past = get_pose_past()

    while abs(get_pose_past()[2] - theta_goal) > (RADS_FOR_EQUALITY):
        velocity_controller(
            left_motor,
            right_motor,
            pose_past[0],
            pose_past[1],
            theta_goal,
            is_turning=True,
        )


def move_forward(left_motor, right_motor, x_goal, y_goal, theta_goal):
    initial_time = time()
    while (
        sqrt((x_goal - get_pose_past()[0]) ** 2 + (y_goal - get_pose_past()[1]) ** 2)
        > DISTANCE_FOR_EQUALITY
        and time() - initial_time < TIME_TO_RECALCULATE_PRIORITIES
    ):

        velocity_controller(
            left_motor,
            right_motor,
            x_goal,
            y_goal,
            theta_goal,
            is_turning=False,
        )

        if check_give_up_on_goal(get_pose_past(), get_goals_reached()):
            return -1
        if get_avoidance_in_progress():
            return -2

    if (
        sqrt((x_goal - get_pose_past()[0]) ** 2 + (y_goal - get_pose_past()[1]) ** 2)
        > DISTANCE_FOR_EQUALITY
    ):
        return -3
    return 0
