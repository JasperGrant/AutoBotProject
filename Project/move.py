#!/usr/bin/env python3

# Main Control Loop for the Gerard SLAM project
# Jasper Grant and Michael MacGillivray
# 2024-04-05

import math as m
from time import sleep, time
from ev3dev2.button import Button
from motion_controller import velocity_controller
from sensor import (
    cardinal_direction_sensor_scan,
    reset_servo,
    wall_identification,
    point_map,
    get_vertical_line,
)

# Motor inputs
from ev3dev2.motor import (
    MoveTank,
    LargeMotor,
    OUTPUT_A,
    OUTPUT_D,
)

# # Sensor inputs
# from ev3dev2.sensor import INPUT_1, INPUT_4

# # Sensor types
# from ev3dev2.sensor.lego import ColorSensor


# Set up robot as tank along with sensors
robot = MoveTank(OUTPUT_D, OUTPUT_A)
left_motor = LargeMotor(OUTPUT_D)
right_motor = LargeMotor(OUTPUT_A)

# Set up buttons
button = Button()

PI = 3.14159

pose_past = [50, 0, PI / 2]
# thresholds for setpoint tracker
NUM_DEGREES_FOR_EQUALITY = 3 * PI / 180
DISTANCE_FOR_EQUALITY = 5
WRONG_DIRECTION_LIMIT = 10

# Inital test conditons for waypoints. Will be updated to be dynamic
DEPTH_GOAL = 20
WIDTH_GOAL = 100


def move_robot():
    # Move in a square pattern
    i = 0

    # reset servo motor to zero
    reset_servo()

    map_file = open("map.txt", "w")
    map_file.write("")
    map_file.close()

    while i < 12:
        print("segment", i)
        y_goal = [30, 60, 90, 140, 140, 140, 140, 90, 60, 30, 0]

        x_goal = [50, 50, 50, 50, 80, 110, 140, 140, 140, 140, 140]
        # y_goal = [30, 60, 60, 30, 0]

        # x_goal = [30, 30, 90, 90, 90]
        # x_goal = 30
        # y_goal = 0

        # theta_goal = [0, -PI / 2, -PI, PI / 2, 0]

        theta_goal = [
            PI / 2,
            PI / 2,
            PI / 2,
            PI / 2,
            0,
            0,
            0,
            -PI / 2,
            -PI / 2,
            -PI / 2,
            -PI / 2,
        ]

        # theta_goal = [PI / 2, PI / 2, 0, -PI / 2, -PI / 2]
        if pose_past[2] - theta_goal[i] > 10 * PI / 180:
            turn(left_motor, right_motor, theta_goal[i])
            print("turning")
        move_forward(left_motor, right_motor, x_goal[i], y_goal[i], theta_goal[i])

        left_motor.stop()
        right_motor.stop()

        sleep(1)
        cardinal_direction_sensor_scan(60, 5, pose_past)
        wall_identification(point_map)
        range_sense = get_vertical_line(point_map[2], "L")[0][0]

        i += 1
    left_motor.stop()
    right_motor.stop()


def turn(left_motor, right_motor, theta_goal):
    global pose_past
    while abs(pose_past[2] - theta_goal) > (NUM_DEGREES_FOR_EQUALITY):
        pose_past = velocity_controller(
            left_motor,
            right_motor,
            pose_past[0],
            pose_past[1],
            theta_goal,
            pose_past,
            is_turning=True,
        )


def move_forward(left_motor, right_motor, x_goal, y_goal, theta_goal):
    global pose_past

    wrong_direction_count = 0

    while (
        m.sqrt((x_goal - pose_past[0]) ** 2 + (y_goal - pose_past[1]) ** 2)
        > DISTANCE_FOR_EQUALITY
    ):
        pre_dist = m.sqrt((x_goal - pose_past[0]) ** 2 + (y_goal - pose_past[1]) ** 2)
        pose_past = velocity_controller(
            left_motor,
            right_motor,
            x_goal,
            y_goal,
            theta_goal,
            pose_past,
            is_turning=False,
        )
        post_dist = m.sqrt((x_goal - pose_past[0]) ** 2 + (y_goal - pose_past[1]) ** 2)

        if pre_dist < post_dist:
            wrong_direction_count += 1
        if wrong_direction_count > WRONG_DIRECTION_LIMIT:
            print("Goal given up on")
            return False


def main():
    while not button.any():
        if not move_robot():
            break


if __name__ == "__main__":
    main()
