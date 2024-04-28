#!/usr/bin/env python3

# Main Behavioral Architecture for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-13

from math import pi
from time import time, sleep
from ev3dev2.button import Button
from detect import (
    get_avoidance_in_progress,
    set_avoidance_in_progress,
)
from avoid import (
    follow_wall,
    set_wall_following_direction,
)
from goals import (
    get_goals_reached,
    increment_goals_reached,
)
from sensor import (
    reset_servo,
    cardinal_direction_sensor_scan,
    wall_identification,
)
from move import (
    move_forward,
    turn,
)
from goals import (
    get_x_goal,
    get_y_goal,
    get_theta_goal,
)
from odometry import get_pose_past, set_pose_past, left_motor, right_motor
from EKF import update_state, get_pred_covariance


button = Button()

WAYPOINT_FOLLOW_CONSTANT_PRIORITY = 5
SCAN_SLOPE_PRIORITY = 0.8
OBSTACLE_NOT_DETECTED_CONSTANT_PRIORITY = 3
OBSTACLE_DETECTED_CONSTANT_PRIORITY = 7
OBJECT_DETECTED_CONSTANT_SCAN_PRIORITY = 8
OBSTACLE_DETECTED_SCAN_FREQUENCY = 50
SCAN_PRIORITY_MAX = 10


def get_distance_since_last_scan():
    # Time since last scan in seconds
    return time() - time_since_last_scan


def waypoint_follow():
    pose_past = get_pose_past()
    goals_reached = get_goals_reached()
    theta_goal = get_theta_goal(goals_reached)
    # TODO: Make move_forward and turn interruptible
    if abs(pose_past[2] - theta_goal) > 45 * pi / 180:
        turn_result = turn(left_motor, right_motor, theta_goal)
        if turn_result == -2:
            left_motor.stop()
            right_motor.stop()
            return -3
    move_result = move_forward(
        left_motor,
        right_motor,
        get_x_goal(goals_reached),
        get_y_goal(goals_reached),
        theta_goal,
    )
    if move_result == -2:
        left_motor.stop()
        right_motor.stop()
        return -2
    if move_result == -1:
        left_motor.stop()
        right_motor.stop()
        return -1
    if move_result == 0:
        increment_goals_reached()
        left_motor.stop()
        right_motor.stop()
        return 0
    if move_result == -3:
        left_motor.stop()
        right_motor.stop()
        return -3


def waypoint_follow_priority():
    return "follow", WAYPOINT_FOLLOW_CONSTANT_PRIORITY, waypoint_follow


def scan():
    left_motor.on(speed=0)
    right_motor.on(speed=0)
    point_map = cardinal_direction_sensor_scan(60, 5, get_pose_past())
    possible_corner = wall_identification(point_map, get_pose_past())
    # Update POSE If Applicable
    shift_string = "No state shift\n"
    if possible_corner is not None:
        print("Corner Detected: ", possible_corner[0], possible_corner[1])
        # Update the pose,
        prev_state = get_pose_past()
        state = update_state(
            possible_corner[0],
            get_pred_covariance(),
            possible_corner[1],
            prev_state,
        )
        shift_string = (
            "State shifted from "
            + str(prev_state[0])
            + ","
            + str(prev_state[1])
            + ","
            + str(prev_state[2])
            + " to "
            + str(state[0])
            + ","
            + str(state[1])
            + ","
            + str(state[2])
            + ","
            "\n"
        )
        point_map = [
            [
                (
                    point[0] + state[0] - prev_state[0],
                    point[1] + state[1] - prev_state[1],
                )
                for point in direction
            ]
            for direction in point_map
        ]

        set_pose_past([state[0], state[1], prev_state[2]])

    self_distance_to_goal = (
        (get_x_goal(get_goals_reached()) - get_pose_past()[0]) ** 2
        + (get_y_goal(get_goals_reached()) - get_pose_past()[1]) ** 2
    ) ** 0.5

    for group in point_map:
        for point in group:

            distance_to_goal = (
                (point[0] - get_x_goal(get_goals_reached())) ** 2
                + (point[1] - get_y_goal(get_goals_reached())) ** 2
            ) ** 0.5
            if distance_to_goal < 10 and self_distance_to_goal < 20:
                increment_goals_reached()
                break

    print(shift_string)
    # Save the point map to a file
    for i, direction in enumerate(["R", "U", "L", "D"]):
        points_file = open("map.csv", "a")
        for point in point_map[i]:
            points_file.write(
                direction + "," + str(point[0]) + "," + str(point[1]) + "\n"
            )
        points_file.close()
    print("Scan Complete")

    global time_since_last_scan
    time_since_last_scan = time()


def scan_priority():
    return (
        "scan",
        min(
            (
                OBJECT_DETECTED_CONSTANT_SCAN_PRIORITY
                if get_avoidance_in_progress()
                and get_distance_since_last_scan() > OBSTACLE_DETECTED_SCAN_FREQUENCY
                else SCAN_SLOPE_PRIORITY * get_distance_since_last_scan()
            ),
            SCAN_PRIORITY_MAX,
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

    # Clear the pose file
    pose_file = open("pose.csv", "w")
    pose_file.write("")
    pose_file.close()

    while not button.any():
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
