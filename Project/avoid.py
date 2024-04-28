#!/usr/bin/env python3

# Obstacle Avoidance functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-13

from math import degrees, pi, cos, sin
from time import time, sleep
from EV3_math_modules import clamp, circle_minus
from detect import (
    get_avoidance_ultrasonic_distance,
    get_bumpers_pressed,
    set_bumpers_pressed,
    is_object_detected,
    move_avoidance_servo_to_angle,
    AVOIDANCE_SERVO_RIGHT_MAX,
    AVOIDANCE_SERVO_LEFT_MAX,
)
from move import turn
from sensor import SIX_FEET

from goals import (
    get_goal_angle,
    get_goal_distance,
    check_give_up_on_goal,
    get_goals_reached,
)

from odometry import get_pose_past, left_motor, right_motor, BASE_WIDTH

MOTOR_BASE_SPEED = 8
MOTOR_HIGH = 10
MOTOR_LOW = 5

k_p = 0.1
k_d = 0.07

WALL_DISTANCE = 15

FRONT_SENSOR_LIMIT = 50

CLEAR_PATH_LIMIT = 25

AVOIDANCE_SERVO_OFFSET = 10

OBSTACLE_NEXT_TO_WALL_DISTANCE = 40

wall_following_direction = None

prev_error = 0

num_turns = 0


def increment_num_turns():
    global num_turns
    num_turns += 1


def reset_num_turns():
    global num_turns
    num_turns = 0


def get_num_turns():
    return num_turns


def get_wall_following_direction():
    return wall_following_direction


def set_wall_following_direction(direction):
    global wall_following_direction
    wall_following_direction = direction


def follow_wall(direction="L"):
    if get_wall_following_direction() is None:
        reset_num_turns()
        # Read survey angle

        pose_past = get_pose_past()

        sleep(5)

        # Transformation matrix here
        L_min = float("inf")
        R_min = float("inf")

        if get_bumpers_pressed():
            # Backup
            left_motor.on(speed=-MOTOR_LOW)
            right_motor.on(speed=-MOTOR_LOW)
            sleep(3)
            left_motor.stop()
            right_motor.stop()

        object_in_front = False
        for i in range(90, 0, -10):
            move_avoidance_servo_to_angle(i)
            distance = get_avoidance_ultrasonic_distance()
            if i < 25 and distance < FRONT_SENSOR_LIMIT:
                object_in_front = True
            if distance < 40:
                unclamped_point = (
                    distance * cos(i + pose_past[2])
                    + pose_past[0]
                    + cos(pose_past[2]) * AVOIDANCE_SERVO_OFFSET,
                    distance * sin(i + pose_past[2])
                    + pose_past[1]
                    + sin(pose_past[2]) * AVOIDANCE_SERVO_OFFSET,
                )
                map_file = open("map.csv", "a")
                map_file.write(
                    "F,"
                    + str(unclamped_point[0])
                    + ","
                    + str(unclamped_point[1])
                    + "\n"
                )
                map_file.close()
            distance = clamp(
                distance,
                0,
                get_goal_distance(
                    (get_pose_past()[0], get_pose_past()[1]), get_goals_reached()
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
            L_min = min(L_min, get_goal_distance(point, get_goals_reached()))
        for i in range(0, -90, -10):
            move_avoidance_servo_to_angle(i)
            distance = get_avoidance_ultrasonic_distance()
            if i > -25 and distance < FRONT_SENSOR_LIMIT:
                object_in_front = True
            if distance < 40:
                unclamped_point = (
                    distance * cos(i + pose_past[2])
                    + pose_past[0]
                    + cos(pose_past[2]) * AVOIDANCE_SERVO_OFFSET,
                    distance * sin(i + pose_past[2])
                    + pose_past[1]
                    + sin(pose_past[2]) * AVOIDANCE_SERVO_OFFSET,
                )
                map_file = open("map.csv", "a")
                map_file.write(
                    "F,"
                    + str(unclamped_point[0])
                    + ","
                    + str(unclamped_point[1])
                    + "\n"
                )
                map_file.close()
            distance = clamp(
                distance,
                0,
                get_goal_distance(
                    (get_pose_past()[0], get_pose_past()[1]), get_goals_reached()
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
            R_min = min(R_min, get_goal_distance(point, get_goals_reached()))

        direction = "L" if L_min < R_min else "R"

        # Override direction if too close to wall
        if pose_past[0] < 30 and 0 < pose_past[2] < pi:
            direction = "R"
        if pose_past[0] > SIX_FEET - 30 and -pi < pose_past[2] < 0:
            direction = "L"
        if pose_past[1] > SIX_FEET - 30 and -pi / 2 < pose_past[2] < pi / 2:
            direction = "R"

        if get_bumpers_pressed():
            direction = "R" if get_bumpers_pressed() == "L" else "L"
            object_in_front = True

        avoidance_turn = pi / 2 if not get_bumpers_pressed() else pi / 4
        if object_in_front:
            goal = pose_past[2] + (
                avoidance_turn if direction == "L" else -avoidance_turn
            )
            turn(left_motor, right_motor, circle_minus(goal))
            left_motor.stop()
            right_motor.stop()

        set_wall_following_direction(direction)
        set_bumpers_pressed(False)

    else:
        if get_bumpers_pressed():
            set_wall_following_direction(None)
            left_motor.stop()
            right_motor.stop()

            return False

    pose_past = get_pose_past()

    direction = get_wall_following_direction()

    move_avoidance_servo_to_angle(-90 if direction == "L" else 90)

    survey_angle_reading = get_avoidance_ultrasonic_distance() - BASE_WIDTH / 2

    error = clamp(WALL_DISTANCE - survey_angle_reading, -20, 20)

    global prev_error
    derivative_error = error - prev_error

    motor_input_change = clamp(k_p * error + k_d * derivative_error, -4, 4)

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

    print(motor_speeds)
    # Handle wall in front
    if wall_in_front:
        motor_speeds = (
            (-MOTOR_LOW, MOTOR_LOW) if direction == "L" else (MOTOR_LOW, -MOTOR_LOW)
        )
    left_motor.on(speed=motor_speeds[0])
    right_motor.on(speed=motor_speeds[1])

    # pose_past = get_pose_past()
    # goals_reached = get_goals_reached()

    check_give_up_on_goal(pose_past, get_goals_reached())
    # If goal angle is withing front servo range
    # Turn to goal angle, check if blocked
    # Else return False
    goal_angle = get_goal_angle(pose_past, get_goals_reached())

    # Wrong direction case
    if (goal_angle > 135 or goal_angle < -135) and get_num_turns() < 2:
        # Flip 180 degrees
        increment_num_turns()
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
        detected = False
        for i in range(int(goal_angle) - 15, int(goal_angle) + 15, 10):
            move_avoidance_servo_to_angle(i)
            if is_object_detected(CLEAR_PATH_LIMIT):
                detected = True
        return not detected
    return False
