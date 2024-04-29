#!/usr/bin/env python3

# Solution for Q2 of A1 MECH-6905
# Jasper Grant and Michael MacGillivray


import math as m
from time import sleep, time
from Q1 import get_wheel_velocity, pose_past, BASE_WIDTH, clamp

# Motor inputs
from ev3dev2.motor import (
    MoveTank,
    LargeMotor,
    MediumMotor,
    OUTPUT_A,
    OUTPUT_D,
    OUTPUT_B,
)

# Sensor inputs
from ev3dev2.sensor import INPUT_3, INPUT_4

# Sensor types
from ev3dev2.sensor.lego import UltrasonicSensor

# Button
from ev3dev2.button import Button

# Set up robot as tank along with sensors
robot = MoveTank(OUTPUT_D, OUTPUT_A)
# mdiff = MoveDifferential(OUTPUT_A, OUTPUT_D, EV3Tire, 100.0) #Need to change measurments
left_motor = LargeMotor(OUTPUT_D)
right_motor = LargeMotor(OUTPUT_A)
servo = MediumMotor(OUTPUT_B)
ultrasonic_sensor = UltrasonicSensor(address=INPUT_3)

# Set up buttons
button = Button()

WALL_DISTANCE = 20

# Motor speeds for default turn severity
MOTOR_HIGH = 10
MOTOR_LOW = 8

ABSOLUTE_SURVEY_ANGLE = 25

# Proportional controller gain
k = 0.1

MOTOR_BASE_SPEED = 10


def move_servo_to_angle(angle):
    servo.on_to_position(10, angle)


def update_odometry(left_motor, right_motor):
    global pose_past
    # Pull velocity and time step values from encoders (Needs work)
    l_velo_current, r_velo_current, delta_t = get_wheel_velocity(
        left_motor, right_motor
    )

    # Calc linear and angular velocity
    x_dot = m.cos(pose_past[2]) * ((l_velo_current + r_velo_current) / 2)
    y_dot = m.sin(pose_past[2]) * ((l_velo_current + r_velo_current) / 2)
    omega = (r_velo_current - l_velo_current) / BASE_WIDTH

    pose_past[2] = pose_past[2] + omega * delta_t
    pose_past[0] = pose_past[0] + x_dot * delta_t
    pose_past[1] = pose_past[1] + y_dot * delta_t


def follow_wall():
    update_odometry(left_motor, right_motor)

    # Read survey angle
    move_servo_to_angle(0)
    while servo.is_running:
        sleep(0.1)
        pass
    survey_angle_reading = ultrasonic_sensor.distance_centimeters

    error = clamp(WALL_DISTANCE - survey_angle_reading, -1000, 40)

    motor_input_change = k * error

    # Read in front
    move_servo_to_angle(-90)
    while servo.is_running:
        sleep(0.1)
        pass
    wall_in_front = ultrasonic_sensor.distance_centimeters < WALL_DISTANCE

    # Set motor speeds
    motor_speeds = (
        MOTOR_BASE_SPEED - motor_input_change,
        MOTOR_BASE_SPEED + motor_input_change,
    )
    # Handle wall in front
    if wall_in_front:
        motor_speeds = (-MOTOR_LOW, MOTOR_HIGH)
    robot.on(left_speed=motor_speeds[0], right_speed=motor_speeds[1])


if __name__ == "__main__":
    ultrasonic_sensor.MODE_US_DIST_CM
    servo.reset()

    while not button.any():
        print(servo.degrees)
        follow_wall()
