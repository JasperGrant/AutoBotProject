# Obstacle Detection functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-15

import threading

# Sensor inputs
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_4
from time import sleep, time

# Sensor types
from ev3dev2.sensor.lego import UltrasonicSensor, TouchSensor

from ev3dev2.motor import (
    MediumMotor,
    OUTPUT_B,
)

AVOIDANCE_SERVO_LEFT_MAX = -90
AVOIDANCE_SERVO_RIGHT_MAX = 90
AVOIDANCE_ULTRASONIC_SENSOR_OFSET = 3.5
SCANNING_ANGLE = 50


avoidance_servo = MediumMotor(OUTPUT_B)
avoidance_ultrasonic_sensor = UltrasonicSensor(address=INPUT_1)
L_bumper = TouchSensor(address=INPUT_4)
R_bumper = TouchSensor(address=INPUT_2)

avoidance_in_progress = False

bumpers_pressed = False


def get_avoidance_in_progress():
    return avoidance_in_progress


def set_avoidance_in_progress(value):
    global avoidance_in_progress
    avoidance_in_progress = value


def get_bumpers_pressed():
    return bumpers_pressed


def set_bumpers_pressed(value):
    global bumpers_pressed
    bumpers_pressed = value


def is_bumper_pressed():
    if L_bumper.is_pressed:
        return "L"
    elif R_bumper.is_pressed:
        return "R"
    else:
        return False


avoidance_servo_mutex = threading.Lock()

OBJECT_DETECTION_DISTANCE = 10


def reset_avoidance_servo():
    avoidance_servo.reset()


def get_avoidance_ultrasonic_distance():
    return (
        avoidance_ultrasonic_sensor.distance_centimeters
        - AVOIDANCE_ULTRASONIC_SENSOR_OFSET
    )


def is_object_detected(range=OBJECT_DETECTION_DISTANCE):
    return get_avoidance_ultrasonic_distance() < range


def move_avoidance_servo_to_angle(angle, speed=10):
    with avoidance_servo_mutex:
        avoidance_servo.on_to_position(speed, angle)
        starting_time = time()
        while time() - starting_time < 0.01:
            sleep(0.1)
            if is_object_detected():
                set_avoidance_in_progress(True)
            status = is_bumper_pressed()
            if status:
                set_avoidance_in_progress(True)
                set_bumpers_pressed(status)


def front_sensor_continous_scan():
    reset_avoidance_servo()
    while True:
        if not get_avoidance_in_progress():
            for i in range(-SCANNING_ANGLE, SCANNING_ANGLE, int(SCANNING_ANGLE / 2)):
                move_avoidance_servo_to_angle(i)
            for i in range(SCANNING_ANGLE, -SCANNING_ANGLE, int(-SCANNING_ANGLE / 2)):
                move_avoidance_servo_to_angle(i)
        else:
            sleep(0.5)
            status = is_bumper_pressed()
            if status:
                set_avoidance_in_progress(True)
                set_bumpers_pressed(status)


scanning_thread = threading.Thread(target=front_sensor_continous_scan)
scanning_thread.daemon = True
scanning_thread.start()
