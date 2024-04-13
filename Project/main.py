#!/usr/bin/env python3

# Main Behavioral Architecture for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-13

from math import pi
from ev3dev2.button import Button
from avoid import is_object_detected
from sensor import (
    reset_servo,
    cardinal_direction_sensor_scan,
    wall_identification,
    point_map,
)
from move import pose_past, left_motor, right_motor, move_forward, turn
from time import time

button = Button()

WAYPOINT_FOLLOW_CONSTANT_PRIORITY = 6
SCAN_SLOPE_PRIORITY = 4
OBSTACLE_NOT_DETECTED_CONSTANT_PRIORITY = 2
OBSTACLE_DETECTED_CONSTANT_PRIORITY = 7


def get_distance_since_last_scan():
    # Time since last scan in seconds
    return time() - time_since_last_scan


y_goal = [30, 60, 90, 140, 140, 140, 140, 90, 60, 30, 0]

x_goal = [50, 50, 50, 50, 80, 110, 140, 140, 140, 140, 140]

theta_goal = [
    pi / 2,
    pi / 2,
    pi / 2,
    pi / 2,
    0,
    0,
    0,
    -pi / 2,
    -pi / 2,
    -pi / 2,
    -pi / 2,
]


def waypoint_follow():
    global goals_reached

    # TODO: Make move_forward and turn interruptible
    if pose_past[2] - theta_goal[goals_reached] > 10 * pi / 180:
        turn_result = turn(left_motor, right_motor, theta_goal[goals_reached])
        if turn_result == -2:
            left_motor.stop()
            right_motor.stop()
            return
    move_result = move_forward(
        left_motor,
        right_motor,
        x_goal[goals_reached],
        y_goal[goals_reached],
        theta_goal[goals_reached],
    )
    if move_result == -2:
        left_motor.stop()
        right_motor.stop()
        return
    if move_result == -1:
        return
    if move_result == 0:
        goals_reached += 1

    left_motor.stop()
    right_motor.stop()


def waypoint_follow_priority():
    return "follow", WAYPOINT_FOLLOW_CONSTANT_PRIORITY, waypoint_follow


def scan():
    cardinal_direction_sensor_scan(60, 5, pose_past)
    wall_identification(point_map)
    pass


def scan_priority():
    return "scan", SCAN_SLOPE_PRIORITY * get_distance_since_last_scan(), scan


def obstacle_avoid():
    while 1:
        print("Obstacle detected")
    pass


def obstacle_avoid_priority():
    return (
        "avoid",
        (
            OBSTACLE_DETECTED_CONSTANT_PRIORITY
            if is_object_detected()
            else OBSTACLE_NOT_DETECTED_CONSTANT_PRIORITY
        ),
        obstacle_avoid,
    )


# Initialize the time since last scan
time_since_last_scan = time()


# Priority functions return their name, a priority value, and the function itself

priority_functions = [waypoint_follow_priority, scan_priority, obstacle_avoid_priority]

goals_reached = 0


def arbitrator():

    # reset servo motor to zero
    reset_servo()

    # Clear the map file
    map_file = open("map.txt", "w")
    map_file.write("")
    map_file.close()

    # Clear the behavior file
    behavior_file = open("behavior.txt", "w")
    behavior_file.write("")
    behavior_file.close()

    while not button.any():
        print("Calculating Priorities")
        priorities = [priority() for priority in priority_functions]
        behavior_file = open("behavior.txt", "a")
        behavior_file.write(
            str(priorities[0])
            + ","
            + str(priorities[1])
            + ","
            + str(priorities[2] + "\n")
        )
        behavior_file.close()
        behavior = priorities.sort(key=lambda x: x[1])[-1]
        print(behavior[0])
        behavior[2]()


if __name__ == "__main__":
    arbitrator()
