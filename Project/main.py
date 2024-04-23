#!/usr/bin/env python3

# Main Behavioral Architecture for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-13

from math import pi
from time import time, sleep
from ev3dev2.button import Button
from detect import (
    reset_avoidance_servo,
    get_avoidance_in_progress,
    set_avoidance_in_progress,
)
from avoid import (
    follow_wall,
    get_goals_reached,
    increment_goals_reached,
    set_wall_following_direction,
)
from sensor import (
    reset_servo,
    cardinal_direction_sensor_scan,
    wall_identification,
    point_map,
)
from move import (
    move_forward,
    turn,
    get_x_goal,
    get_y_goal,
    get_theta_goal,
)
from odometry import get_pose_past, set_pose_past, left_motor, right_motor
from EKF import update_state, get_pred_covariance


button = Button()

WAYPOINT_FOLLOW_CONSTANT_PRIORITY = 6
SCAN_SLOPE_PRIORITY = 0.2
OBSTACLE_NOT_DETECTED_CONSTANT_PRIORITY = 2
OBSTACLE_DETECTED_CONSTANT_PRIORITY = 7
OBJECT_DETECTED_CONSTANT_SCAN_PRIORITY = 8
OBSTACLE_DETECTED_SCAN_FREQUENCY = 100


def get_distance_since_last_scan():
    # Time since last scan in seconds
    return time() - time_since_last_scan


def waypoint_follow():
    pose_past = get_pose_past()
    goals_reached = get_goals_reached()
    theta_goal = get_theta_goal()
    # TODO: Make move_forward and turn interruptible
    if abs(pose_past[2] - theta_goal[goals_reached]) > 10 * pi / 180:
        turn_result = turn(left_motor, right_motor, theta_goal[goals_reached])
        if turn_result == -2:
            left_motor.stop()
            right_motor.stop()
            return -3
    move_result = move_forward(
        left_motor,
        right_motor,
        get_x_goal()[goals_reached],
        get_y_goal()[goals_reached],
        get_theta_goal()[goals_reached],
    )
    if move_result == -2:
        print("Obstacle Detected")
        left_motor.stop()
        right_motor.stop()
        return -2
    if move_result == -1:
        left_motor.stop()
        right_motor.stop()
        return -1
    if move_result == 0:
        print("Goal Reached")
        increment_goals_reached()
        left_motor.stop()
        right_motor.stop()
        return 0


def waypoint_follow_priority():
    return "follow", WAYPOINT_FOLLOW_CONSTANT_PRIORITY, waypoint_follow


def scan():
    left_motor.on(speed=0)
    right_motor.on(speed=0)
    cardinal_direction_sensor_scan(60, 5, get_pose_past())
    wall_identification(point_map)

    # Update POSE If Applicable
    if True:
        # Update the pose,
        state = update_state([0, 180], get_pred_covariance(), [0, 180], get_pose_past())
        set_pose_past(state)

    global time_since_last_scan
    time_since_last_scan = time()


def scan_priority():
    return (
        "scan",
        (
            OBJECT_DETECTED_CONSTANT_SCAN_PRIORITY
            if get_avoidance_in_progress()
            and get_distance_since_last_scan() > OBSTACLE_DETECTED_SCAN_FREQUENCY
            else SCAN_SLOPE_PRIORITY * get_distance_since_last_scan()
        ),
        scan,
    )


def obstacle_avoid():
    set_avoidance_in_progress(True)
    # Fully scan object
    # Decide which direction to go
    # Wall follow in that direction

    if follow_wall("L"):
        left_motor.on(speed=5)
        right_motor.on(speed=5)
        sleep(1)
        left_motor.stop()
        right_motor.stop()
        set_avoidance_in_progress(False)
        set_wall_following_direction(None)


def obstacle_avoid_priority():
    return (
        "avoid",
        (
            OBSTACLE_DETECTED_CONSTANT_PRIORITY
            if get_avoidance_in_progress()
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
    # Clear the map file
    map_file = open("map.csv", "w")
    map_file.write("")
    map_file.close()

    # Clear the behavior file
    behavior_file = open("behavior.csv", "w")
    behavior_file.write("")
    behavior_file.close()

    while not button.any():
        print("Calculating Priorities")
        print("Object? ", get_avoidance_in_progress())
        priorities = [priority() for priority in priority_functions]
        behavior_file = open("behavior.csv", "a")
        behavior_file.write(
            str(time())
            + ","
            + str(priorities[0][1])
            + ","
            + str(priorities[1][1])
            + ","
            + str(priorities[2][1])
            + "\n",
        )
        behavior_file.close()
        behavior = max(priorities, key=lambda x: x[1])
        behavior[2]()


if __name__ == "__main__":
    arbitrator()
