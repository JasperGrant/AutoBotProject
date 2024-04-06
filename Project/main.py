# Main Control Loop for the Gerard SLAM project
# Jasper Grant and Michael MacGillivray
# 2024-04-05

import math as m
from time import sleep, time

# Motor inputs
from ev3dev2.motor import (
    MoveTank,
    LargeMotor,
    OUTPUT_A,
    OUTPUT_D,
)

# Sensor inputs
from ev3dev2.sensor import INPUT_1, INPUT_4

# Sensor types
from ev3dev2.sensor.lego import ColorSensor

# Button
from ev3dev2.button import Button

# Set up robot as tank along with sensors
robot = MoveTank(OUTPUT_D, OUTPUT_A)
left_motor = LargeMotor(OUTPUT_D)
right_motor = LargeMotor(OUTPUT_A)

# Set up buttons
button = Button()


def main():
    while not button.any():
        pass


if __name__ == "__main__":
    main()
