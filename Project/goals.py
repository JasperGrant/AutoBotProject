#!/usr/bin/env python3

# Goal functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-23

from math import degrees, atan2, sqrt
from EV3_math_modules import circle_minus
from math import cos, sin


goals_reached = 0


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


def get_goal_directions(i):
    return goal_directions[i]


def get_x_goal(i):
    return x_goal[i]


def get_y_goal(i):
    return y_goal[i]


def get_theta_goal(i):
    return theta_goal[i]


def get_goals_reached():
    return goals_reached


def increment_goals_reached():
    global goals_reached
    goals_reached += 1
    return goals_reached


def get_goal_angle(pose_past, goals_reached, offset=10):
    return degrees(
        circle_minus(
            atan2(
                get_y_goal(goals_reached) - pose_past[1] + cos(pose_past[2]) * offset,
                get_x_goal(goals_reached) - pose_past[0] + sin(pose_past[2]) * offset,
            )
            - pose_past[2]
        )
    )


def get_goal_distance(point, goals_reached):
    return sqrt(
        (get_y_goal(goals_reached) - point[1]) ** 2
        + (get_x_goal(goals_reached) - point[0]) ** 2
    )


def check_give_up_on_goal(pose_past, goals_reached):
    if get_goal_directions(goals_reached) == "U":
        if pose_past[1] >= get_y_goal(goals_reached):
            increment_goals_reached()
            return True
    elif get_goal_directions(goals_reached) == "D":
        if pose_past[1] <= get_y_goal(goals_reached):
            increment_goals_reached()
            return True
    elif get_goal_directions(goals_reached) == "R":
        if pose_past[0] >= get_x_goal(goals_reached):
            increment_goals_reached()
            return True
    else:
        print("Error: Invalid goal direction")
        exit(1)
    return False
