#!/usr/bin/env python3

# Solution for Q1 of A1 MECH-6905
# Jasper Grant and Michael MacGillivray

# Motor inputs
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_D
# Sensor inputs
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
# Sensor types
from ev3dev2.sensor.lego import ColorSensor, GyroSensor
# Button
from ev3dev2.button import Button

# Set up robot as tank along with sensors
robot = MoveTank(OUTPUT_A, OUTPUT_D)
robot._gyro = GyroSensor(address = INPUT_2)
robot._cs = ColorSensor(address = INPUT_1)

# Set up buttons
button = Button()

state = "Line Following"

def follow_line():
    pass

def find_line():
    pass

while not button.any():
    if state == "Line Following":
        follow_line()
    elif state == "Line Finding":
        find_line()
    else:
        print("Invalid state")
        exit(-1)
