#!/usr/bin/env python3

# Solution for Q1 of A1 MECH-6905
# Jasper Grant and Michael MacGillivray
import math as m
from ev3dev2.wheel import EV3Tire

# Motor inputs
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_D, MoveDifferential, SpeedRPM
# Sensor inputs
from ev3dev2.sensor import INPUT_1, INPUT_4
# Sensor types
from ev3dev2.sensor.lego import ColorSensor, GyroSensor
# Button
from ev3dev2.button import Button

# Set up robot as tank along with sensors
robot = MoveTank(OUTPUT_A, OUTPUT_D)
mdiff = MoveDifferential(OUTPUT_A, OUTPUT_D, EV3Tire, 100.0) #Need to change measurments
gyro = GyroSensor(address = INPUT_4)
cs = ColorSensor(address = INPUT_1)

# Set up buttons
button = Button()

# Assuming we are on a line to start

# Lught sensor value representing threshold between tape and floor
THRESHOLD = 30

# Motor speeds for default turn severity
MOTOR_HIGH = 10
MOTOR_LOW = 8

# Number of readings for the robot to think it is off of the line
OFFLINE_LIMIT = 100

# Define PI
PI = 3.141

def follow_line():
    offline_readings = 0
    while(offline_readings < OFFLINE_LIMIT):
        print(offline_readings)
        if cs.reflected_light_intensity < THRESHOLD:
            offline_readings = 0
            # Go but a little right
            robot.on(left_speed=MOTOR_HIGH, right_speed=MOTOR_LOW)
        else:
            offline_readings += 1
            # Go but a little left
            robot.on(left_speed=MOTOR_LOW, right_speed=MOTOR_HIGH)
    
    

def find_line():
    turn(90)
    run_straight(100, 0)


def turn(degrees):
    mdiff.turn_to_angle(SpeedRPM(40), degrees)
     

    #difference = degrees - gyro.angle
    #difference_radians = (difference * PI/180 + PI) % (2 * PI) - PI
    #while(gyro.angle <)
    # initial_angle = gyro.angle
    # if degrees < 0:
    #     while gyro.angle - initial_angle < degrees:
    #         robot.on(left_speed=MOTOR_LOW, right_speed=MOTOR_HIGH)
    # else:
    #     while gyro.angle - initial_angle < degrees:
    #         robot.on(left_speed=MOTOR_HIGH, right_speed=MOTOR_LOW)

def run_straight(x_dist, y_dist):
        # mdiff = MoveDifferential(OUTPUT_A, OUTPUT_D)
        # Use odometry to drive to specific coordinates
        mdiff.odometry_start()
        mdiff.on_to_coordinates(SpeedRPM(40), x_dist, y_dist)
        mdiff.odometry_stop()


while not button.any():
    follow_line()
    robot.off()
    # exit()
    find_line()
    exit()
# print(robot._gyro.angle_and_rate)
# robot.turn_degrees(speed=5, target_angle=1)
# while(1):
#     print(robot._gyro.angle_and_rate)