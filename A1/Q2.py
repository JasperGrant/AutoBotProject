#!/usr/bin/env python3

# Solution for Q2 of A1 MECH-6905
# Jasper Grant and Michael MacGillivray


import math as m
from time import sleep, time

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
from ev3dev2.sensor.lego import GyroSensor, UltrasonicSensor

# Button
from ev3dev2.button import Button

# Set up robot as tank along with sensors
robot = MoveTank(OUTPUT_D, OUTPUT_A)
# mdiff = MoveDifferential(OUTPUT_A, OUTPUT_D, EV3Tire, 100.0) #Need to change measurments
left_motor = LargeMotor(OUTPUT_D)
right_motor = LargeMotor(OUTPUT_A)
servo = MediumMotor(OUTPUT_B)
gyro = GyroSensor(address=INPUT_4)
ultrasonic_sensor = UltrasonicSensor(address=INPUT_3)

# Set up buttons
button = Button()

WALL_DISTANCE = 40

# Motor speeds for default turn severity
MOTOR_HIGH = 10
MOTOR_LOW = 8


def move_servo_to_angle(servo, angle):
    servo.on_to_position(10, angle)


def follow_wall():
    move_servo_to_angle(servo, -gyro.angle - 45)
    if not servo.is_running:
        towards_wall = ultrasonic_sensor.distance_centimeters > WALL_DISTANCE
        print(ultrasonic_sensor.distance_centimeters)
        motor_speeds = (
            (MOTOR_LOW, MOTOR_HIGH) if not towards_wall else (MOTOR_HIGH, MOTOR_LOW)
        )
        robot.on(left_speed=motor_speeds[0], right_speed=motor_speeds[1])
    else:
        robot.off()


ultrasonic_sensor.MODE_US_DIST_CM
servo.reset()
gyro.reset()

while not button.any():
    print(servo.degrees)
    follow_wall()
