# Obstacle Detection functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-15

import threading

# Sensor inputs
from ev3dev2.sensor import INPUT_1
from time import sleep, time

# Sensor types
from ev3dev2.sensor.lego import UltrasonicSensor

from ev3dev2.motor import (
    MediumMotor,
    OUTPUT_B,
)

AVOIDANCE_SERVO_LEFT_MAX = -90
AVOIDANCE_SERVO_RIGHT_MAX = 90


avoidance_servo = MediumMotor(OUTPUT_B)
avoidance_ultrasonic_sensor = UltrasonicSensor(address=INPUT_1)

avoidance_in_progress = False


def get_avoidance_in_progress():
    return avoidance_in_progress


def set_avoidance_in_progress(value):
    global avoidance_in_progress
    avoidance_in_progress = value


avoidance_servo_mutex = threading.Lock()

OBJECT_DETECTION_DISTANCE = 20


def reset_avoidance_servo():
    avoidance_servo.reset()


def is_object_detected():
    return avoidance_ultrasonic_sensor.distance_centimeters < OBJECT_DETECTION_DISTANCE


def move_avoidance_servo_to_angle(angle, speed=10):
    with avoidance_servo_mutex:
        avoidance_servo.on_to_position(speed, angle)
        starting_time = time()
        while time() - starting_time < 0.05:
            if is_object_detected():
                set_avoidance_in_progress(True)


def front_sensor_continous_scan():
    reset_avoidance_servo()
    while True:
        if not get_avoidance_in_progress():
            for i in range(-90, 90, 30):
                move_avoidance_servo_to_angle(i)
            for i in range(90, -90, -30):
                move_avoidance_servo_to_angle(i)
        else:
            sleep(1)


scanning_thread = threading.Thread(target=front_sensor_continous_scan)
scanning_thread.daemon = True
scanning_thread.start()
