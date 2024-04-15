#!/usr/bin/env python3

# Main Behavioral Architecture for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-13

from math import pi
from ev3dev2.button import Button
from detect import is_object_detected, reset_avoidance_servo
from avoid import follow_wall
from sensor import (
    reset_servo,
    cardinal_direction_sensor_scan,
    wall_identification,
    point_map,
)
from move import (
    pose_past,
    left_motor,
    right_motor,
    move_forward,
    turn,
    x_goal,
    y_goal,
    theta_goal,
    goals_reached,
)
from time import time

button = Button()

WAYPOINT_FOLLOW_CONSTANT_PRIORITY = 6
SCAN_SLOPE_PRIORITY = 0.2
OBSTACLE_NOT_DETECTED_CONSTANT_PRIORITY = 2
OBSTACLE_DETECTED_CONSTANT_PRIORITY = 7
OBJECT_DETECTED_CONSTANT_SCAN_PRIORITY = 8
OBSTACLE_DETECTED_SCAN_FREQUENCY = 5

avoidance_in_progress = False


def get_distance_since_last_scan():
    # Time since last scan in seconds
    return time() - time_since_last_scan


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
        # y_goal[goals_reached],
        theta_goal[goals_reached],
    )
    if move_result == -2:
        print("Obstacle Detected")
        left_motor.stop()
        right_motor.stop()
        return
    if move_result == -1:
        left_motor.stop()
        right_motor.stop()
        return
    if move_result == 0:
        print("Goal Reached")
        goals_reached += 1
        left_motor.stop()
        right_motor.stop()


def waypoint_follow_priority():
    return "follow", WAYPOINT_FOLLOW_CONSTANT_PRIORITY, waypoint_follow


def scan():
    cardinal_direction_sensor_scan(60, 5, pose_past)
    wall_identification(point_map)
    global time_since_last_scan
    time_since_last_scan = time()


def scan_priority():
    is_object_detected()
    return (
        "scan",
        (
            OBJECT_DETECTED_CONSTANT_SCAN_PRIORITY
            if is_object_detected()
            and get_distance_since_last_scan() > OBSTACLE_DETECTED_SCAN_FREQUENCY
            else SCAN_SLOPE_PRIORITY * get_distance_since_last_scan()
        ),
        scan,
    )


def obstacle_avoid():
    avoidance_in_progress = True
    # Fully scan object
    # Decide which direction to go
    # Wall follow in that direction
    if follow_wall("L"):
        avoidance_in_progress = False


def obstacle_avoid_priority():
    return (
        "avoid",
        (
            OBSTACLE_DETECTED_CONSTANT_PRIORITY
            if avoidance_in_progress or is_object_detected()
            else OBSTACLE_NOT_DETECTED_CONSTANT_PRIORITY
        ),
        obstacle_avoid,
    )


# Initialize the time since last scan
time_since_last_scan = time()


# Priority functions return their name, a priority value, and the function itself

priority_functions = [waypoint_follow_priority, scan_priority, obstacle_avoid_priority]


def arbitrator():

    # reset servo motor to zero
    reset_servo()
    # reset avoidance servo to zero
    reset_avoidance_servo()
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
        print("HELLO", priorities)
        behavior_file = open("behavior.txt", "a")
        behavior_file.write(
            str(priorities[0][1])
            + ","
            + str(priorities[1][1])
            + ","
            + str(priorities[2][1])
            + "\n"
        )
        behavior_file.close()
        behavior = max(priorities, key=lambda x: x[1])

        print(behavior[0])
        behavior[2]()


if __name__ == "__main__":
    arbitrator()
