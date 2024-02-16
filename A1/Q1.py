#!/usr/bin/env python3

# Solution for Q1 of A1 MECH-6905
# Jasper Grant and Michael MacGillivray

# Motor inputs
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_D
# Sensor inputs
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
# Sensor types
from ev3dev2.sensor.lego import ColorSensor, GyroSensor

# Set up robot as tank along with sensors
robot = MoveTank(OUTPUT_A, OUTPUT_D)
robot._gyro = GyroSensor(address = INPUT_2)
robot._cs = ColorSensor(address = INPUT_1)

# Functiom to dead reckon straight distance cm
def dead_reckon_straight(distance):
    pass

# Function to dead reckon turn angle degrees
def dead_reckon_turn(angle):
    pass


while(1):
    robot.on(left_speed=10, right_speed=10)


