#!/usr/bin/env python3

# Obstacle Avoidance functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-13

from math import degrees, pi, atan2, cos, sin
from time import time

from EV3_math_modules import clamp
from detect import (
    avoidance_ultrasonic_sensor,
    is_object_detected,
    move_avoidance_servo_to_angle,
    AVOIDANCE_SERVO_RIGHT_MAX,
    AVOIDANCE_SERVO_LEFT_MAX,
)
from move import get_x_goal, get_y_goal

from odometry import get_pose_past, left_motor, right_motor

MOTOR_BASE_SPEED = 8
MOTOR_HIGH = 10
MOTOR_LOW = 5
k = 0.1

WALL_DISTANCE = 20

goals_reached = 0


def get_goals_reached():
    return goals_reached


def increment_goals_reached():
    global goals_reached
    goals_reached += 1
    return goals_reached


def get_goal_angle(pose_past, goals_reached):
    return degrees(
        atan2(
            get_y_goal()[goals_reached] - pose_past[1],
            get_x_goal()[goals_reached] - pose_past[0],
        )
    ) - degrees(pose_past[2])


def check_distance_to_goal(pose_past, goals_reached):
    if goals_reached > 16:
        return 0
    elif goals_reached <= 5:
        if pose_past[1] > get_y_goal()[goals_reached]:
            return increment_goals_reached()
    elif goals_reached <= 10:
        if pose_past[0] > get_x_goal()[goals_reached]:
            return increment_goals_reached()
    else:
        if pose_past[1] < get_y_goal()[goals_reached]:
            return increment_goals_reached()


def follow_wall(direction="L"):
    print(time())
    # Read survey angle
    print("Following wall")
    move_avoidance_servo_to_angle(-90)
    survey_angle_reading = avoidance_ultrasonic_sensor.distance_centimeters

    error = clamp(WALL_DISTANCE - survey_angle_reading, -1000, 40)

    print("Error: ", error)

    motor_input_change = k * error

    # Read in front
    move_avoidance_servo_to_angle(0)
    wall_in_front = avoidance_ultrasonic_sensor.distance_centimeters < WALL_DISTANCE

    # Set motor speeds
    motor_speeds = (
        MOTOR_BASE_SPEED - motor_input_change,
        MOTOR_BASE_SPEED + motor_input_change,
    )
    # Handle wall in front
    if wall_in_front:
        motor_speeds = (-MOTOR_LOW, MOTOR_LOW)
    # left_motor.on(speed=motor_speeds[0])
    # right_motor.on(speed=motor_speeds[1])

    pose_past = get_pose_past()
    goals_reached = get_goals_reached()

    check_distance_to_goal(pose_past, goals_reached)
    # If goal angle is withing front servo range
    # Turn to goal angle, check if blocked
    # Else return False
    goal_angle = get_goal_angle(pose_past, goals_reached)
    print("Goal angle: ", goal_angle)
    print("Pose: ", degrees(pose_past[2]))

    if (
        AVOIDANCE_SERVO_LEFT_MAX + 20 <= goal_angle
        and goal_angle <= AVOIDANCE_SERVO_RIGHT_MAX - 20
    ):
        detected = False
        for i in range(int(goal_angle) - 15, int(goal_angle) + 15, 10):
            print("Checking angle: ", i)
            move_avoidance_servo_to_angle(i)
            if is_object_detected():
                detected = True
        # for i in range(90, -90, -30):
        #     move_avoidance_servo_to_angle(i)
        #     if is_object_detected():
        #         detected = True
        return not detected

    return False
