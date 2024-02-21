#!/usr/bin/env python3

# Solution for Q1 of A1 MECH-6905
# Jasper Grant and Michael MacGillivray

# Motor inputs
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_D
# Sensor inputs
from ev3dev2.sensor import INPUT_1, INPUT_4
# Sensor types
from ev3dev2.sensor.lego import ColorSensor, GyroSensor
# Button
from ev3dev2.button import Button

# Set up robot as tank along with sensors
robot = MoveTank(OUTPUT_A, OUTPUT_D)
robot._gyro = GyroSensor(address = INPUT_4)
cs = ColorSensor(address = INPUT_1)

# Set up buttons
button = Button()

# Assuming we are on a line to start

# Lught sensor value representing threshold between tape and floor
THRESHOLD = 30

# Motor speeds for default turn severity
MOTOR_HIGH = 10
MOTOR_LOW = 8

def follow_line():
    # TODO: Decide on condition to end line following
    while(condition):
        if cs.reflected_light_intensity < THRESHOLD:
            # Go but a little left
            robot.on(left_speed=MOTOR_HIGH, right_speed=MOTOR_LOW)
        else:
            # Go but a little right
            robot.on(left_speed=MOTOR_LOW, right_speed=MOTOR_HIGH)
    
    

def find_line():
    # TODO: Implement find_line()
    pass

while not button.any():
    follow_line()
    find_line()