#!/usr/bin/env python3

# Obstacle Avoidance functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-13

from math import degrees, pi, atan2, cos, sin, sqrt
from time import time, sleep
from EV3_math_modules import clamp, circle_minus
from detect import (
    get_avoidance_ultrasonic_distance,
    is_object_detected,
    move_avoidance_servo_to_angle,
    AVOIDANCE_SERVO_RIGHT_MAX,
    AVOIDANCE_SERVO_LEFT_MAX,
)
from move import get_x_goal, get_y_goal, turn, get_goal_directions

from odometry import get_pose_past, left_motor, right_motor, BASE_WIDTH

MOTOR_BASE_SPEED = 8
MOTOR_HIGH = 10
MOTOR_LOW = 5

k_p = 0.1
k_i = 0.1
k_d = 0.1

WALL_DISTANCE = 15

FRONT_SENSOR_LIMIT = 50

CLEAR_PATH_LIMIT = 25

AVOIDANCE_SERVO_OFFSET = 10

goals_reached = 0

wall_following_direction = None

prev_error = 0
integral_error = 0


def get_wall_following_direction():
    return wall_following_direction


def set_wall_following_direction(direction):
    global wall_following_direction
    wall_following_direction = direction


def get_goals_reached():
    return goals_reached


def increment_goals_reached():
    global goals_reached
    goals_reached += 1
    return goals_reached


def get_goal_angle(pose_past, goals_reached):
    return degrees(
        circle_minus(
            atan2(
                get_y_goal()[goals_reached] - pose_past[1],
                get_x_goal()[goals_reached] - pose_past[0],
            )
            - pose_past[2]
        )
    )


def get_goal_distance(point, goals_reached):
    return sqrt(
        (get_y_goal()[goals_reached] - point[1]) ** 2
        + (get_x_goal()[goals_reached] - point[0]) ** 2
    )


def check_distance_to_goal(pose_past, goals_reached):
    if get_goal_directions()[goals_reached] == "U":
        if pose_past[1] >= get_y_goal()[goals_reached]:
            increment_goals_reached()
    elif get_goal_directions()[goals_reached] == "D":
        if pose_past[1] <= get_y_goal()[goals_reached]:
            increment_goals_reached()
    elif get_goal_directions()[goals_reached] == "R":
        if pose_past[0] >= get_x_goal()[goals_reached]:
            increment_goals_reached()
    else:
        print("Error: Invalid goal direction")
    pass


def follow_wall(direction="L"):
    if get_wall_following_direction() is None:
        print(time())
        # Read survey angle
        print("Following wall")

        pose_past = get_pose_past()

        count_left = 0
        count_right = 0

        sleep(5)

        # Transformation matrix here
        L_min = float("inf")
        R_min = float("inf")

        for i in range(90, 0, -10):
            move_avoidance_servo_to_angle(i)
            distance = clamp(
                get_avoidance_ultrasonic_distance(),
                0,
                get_goal_distance(
                    (get_pose_past()[0], get_pose_past()[1]), goals_reached
                ),
            )
            point = (
                distance * cos(i + pose_past[2])
                + pose_past[0]
                + cos(pose_past[2]) * AVOIDANCE_SERVO_OFFSET,
                distance * sin(i + pose_past[2])
                + pose_past[1]
                + sin(pose_past[2]) * AVOIDANCE_SERVO_OFFSET,
            )
            L_min = min(L_min, get_goal_distance(point, goals_reached))
        for i in range(0, -90, -10):
            move_avoidance_servo_to_angle(i)
            distance = clamp(
                get_avoidance_ultrasonic_distance(),
                0,
                get_goal_distance(
                    (get_pose_past()[0], get_pose_past()[1]), goals_reached
                ),
            )
            point = (
                distance * cos(i + pose_past[2])
                + pose_past[0]
                + cos(pose_past[2]) * AVOIDANCE_SERVO_OFFSET,
                distance * sin(i + pose_past[2])
                + pose_past[1]
                + sin(pose_past[2]) * AVOIDANCE_SERVO_OFFSET,
            )
            R_min = min(R_min, get_goal_distance(point, goals_reached))

        direction = "L" if L_min < R_min else "R"

        print("Direction: ", direction)

        if direction == "L":
            print("Turning left")
            goal = pose_past[2] + pi / 2
            turn(left_motor, right_motor, circle_minus(goal))
            print("done turning left")
        else:
            print("Turning right")
            turn(left_motor, right_motor, circle_minus(pose_past[2] - pi / 2))
            print("done turning right")

        set_wall_following_direction(direction)

    pose_past = get_pose_past()

    direction = get_wall_following_direction()

    move_avoidance_servo_to_angle(-90 if direction == "L" else 90)

    survey_angle_reading = get_avoidance_ultrasonic_distance() - BASE_WIDTH / 2

    error = clamp(WALL_DISTANCE - survey_angle_reading, -25, 25)

    global prev_error
    dervaitive_error = error - prev_error

    # global integral_error
    # integral_error += error

    # print("Error: ", error)

    motor_input_change = k_p * error + k_d * dervaitive_error  # + k_i * integral_error

    # Set previous error

    prev_error = error

    # Read in front
    move_avoidance_servo_to_angle(0)
    wall_in_front = get_avoidance_ultrasonic_distance() < WALL_DISTANCE

    if direction == "R":
        motor_input_change = -motor_input_change

    # Set motor speeds
    motor_speeds = (
        MOTOR_BASE_SPEED - motor_input_change,
        MOTOR_BASE_SPEED + motor_input_change,
    )
    # Handle wall in front
    if wall_in_front:
        motor_speeds = (
            (-MOTOR_LOW, MOTOR_LOW) if direction == "L" else (MOTOR_LOW, -MOTOR_LOW)
        )
    left_motor.on(speed=motor_speeds[0])
    right_motor.on(speed=motor_speeds[1])

    # pose_past = get_pose_past()
    # goals_reached = get_goals_reached()

    check_distance_to_goal(pose_past, goals_reached)
    # If goal angle is withing front servo range
    # Turn to goal angle, check if blocked
    # Else return False
    goal_angle = get_goal_angle(pose_past, goals_reached)
    print("Goal angle: ", goal_angle)
    print("Pose: ", degrees(pose_past[2]))

    # Wrong direction case
    if goal_angle > 135 or goal_angle < -135:
        # Flip 180 degrees
        turn(left_motor, right_motor, circle_minus(pose_past[2] + pi))
        set_wall_following_direction("L" if direction == "R" else "R")
        return False

    if (
        AVOIDANCE_SERVO_LEFT_MAX + 45 <= goal_angle
        and goal_angle <= AVOIDANCE_SERVO_RIGHT_MAX - 45
    ):
        if (goal_angle < 0 and direction == "L") or (
            goal_angle > 0 and direction == "R"
        ):
            move_avoidance_servo_to_angle(-90 if direction == "L" else 90)
            if is_object_detected():
                return False
        print("Checking for object")
        detected = False
        for i in range(int(goal_angle) - 15, int(goal_angle) + 15, 10):
            print("Checking angle: ", i)
            move_avoidance_servo_to_angle(i)
            if is_object_detected(CLEAR_PATH_LIMIT):
                detected = True
        # for i in range(90, -90, -30):
        #     move_avoidance_servo_to_angle(i)
        #     if is_object_detected():
        #         detected = True
        return not detected
    return False
