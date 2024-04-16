#!/usr/bin/env python3

# Obstacle Avoidance functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-13

from math import degrees, pi, atan2

from EV3_math_modules import clamp
from detect import (
    avoidance_ultrasonic_sensor,
    is_object_detected,
    move_avoidance_servo_to_angle,
    AVOIDANCE_SERVO_RIGHT_MAX,
    AVOIDANCE_SERVO_LEFT_MAX,
)
from move import left_motor, right_motor, pose_past, goals_reached, x_goal, y_goal

MOTOR_BASE_SPEED = 10
MOTOR_HIGH = 10
MOTOR_LOW = 8
k = 0.1

WALL_DISTANCE = 20


def get_goal_angle():
    return degrees(
        atan2(
            y_goal[goals_reached] - pose_past[1], x_goal[goals_reached] - pose_past[0]
        )
    )


def follow_wall(direction="L"):
    # Read survey angle
    print("Following wall")
    move_avoidance_servo_to_angle(-70)
    survey_angle_reading = avoidance_ultrasonic_sensor.distance_centimeters

    error = clamp(WALL_DISTANCE - survey_angle_reading, -1000, 40)

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
    left_motor.on(speed=motor_speeds[0])
    right_motor.on(speed=motor_speeds[1])
    # If goal angle is withing front servo range
    # Turn to goal angle, check if blocked
    # Else return False
    move_avoidance_servo_to_angle((get_goal_angle() - pose_past[2]))
    # Return true if wall is no longer in goal direction
    return not is_object_detected()
