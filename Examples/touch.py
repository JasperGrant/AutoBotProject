#!/usr/bin/env python3

from ev3dev2.sensor import INPUT_2
from ev3dev2.sensor.lego import TouchSensor
from time import sleep

touch_sensor = TouchSensor(INPUT_2)

while True:
    print(touch_sensor.is_pressed)
    sleep(0.5)
