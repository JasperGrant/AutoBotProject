# Obstacle Detection functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-15

import threading

# Sensor inputs
from ev3dev2.sensor import INPUT_1
from time import sleep

# Sensor types
from ev3dev2.sensor.lego import UltrasonicSensor

from ev3dev2.motor import (
    MediumMotor,
    OUTPUT_B,
)


avoidance_servo = MediumMotor(OUTPUT_B)
avoidance_ultrasonic_sensor = UltrasonicSensor(address=INPUT_1)

avoidance_in_progress = False

OBJECT_DETECTION_DISTANCE = 10


def reset_avoidance_servo():
    avoidance_servo.reset()


def is_object_detected():
    return avoidance_ultrasonic_sensor.distance_centimeters < OBJECT_DETECTION_DISTANCE


def move_avoidance_servo_to_angle(angle, speed=10, callback=lambda: sleep(0.1)):
    while avoidance_servo.is_running:
        if callback():
            avoidance_servo.off()
            return True
    avoidance_servo.on_to_position(speed, angle)
    while avoidance_servo.is_running:
        if callback():
            avoidance_servo.off()
            return True


def front_sensor_continous_scan():
    while True:
        if not avoidance_in_progress:
            avoidance_servo.on_to_position(10, 45)
            sleep(1)
            avoidance_servo.on_to_position(10, -45)
            sleep(1)


scanning_thread = threading.Thread(target=front_sensor_continous_scan)
scanning_thread.start()
