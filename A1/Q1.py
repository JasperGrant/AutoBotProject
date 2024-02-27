#!/usr/bin/env python3

# Solution for Q1 of A1 MECH-6905
# Jasper Grant and Michael MacGillivray
import math as m
from ev3dev2.wheel import EV3Tire
from time import sleep, time

# Motor inputs
from ev3dev2.motor import MoveTank, LargeMotor, OUTPUT_A, OUTPUT_D, MoveDifferential, SpeedRPM
# Sensor inputs
from ev3dev2.sensor import INPUT_1, INPUT_4
# Sensor types
from ev3dev2.sensor.lego import ColorSensor, GyroSensor
# Button
from ev3dev2.button import Button

# Set up robot as tank along with sensors
robot = MoveTank(OUTPUT_A, OUTPUT_D)
# mdiff = MoveDifferential(OUTPUT_A, OUTPUT_D, EV3Tire, 100.0) #Need to change measurments
left_motor = LargeMotor(OUTPUT_D)
right_motor = LargeMotor(OUTPUT_A)
gyro = GyroSensor(address = INPUT_4)
cs = ColorSensor(address = INPUT_1)

# Set up buttons
button = Button()

# Assuming we are on a line to start

# Lught sensor value representing threshold between tape and floor
THRESHOLD_EDGE = 30
THRESHOLD_SEARCH = 10

# Motor speeds for default turn severity
MOTOR_HIGH = 10
MOTOR_LOW = 8

# Number of readings for the robot to think it is off of the line
OFFLINE_LIMIT = 100

# Define PI
PI = 3.141

#Velocity controller Gains
K_RHO = 1
K_ALPHA = 2
K_BETA = -1
K_P = 2

#EV3 Parameters
BASE_WIDTH = 0.145 #mm
TIRE_DIAMETER = 0.058 #mm

#Robot Pose (x, y, theta)
pose_past = [0, 0, 0]  # Initial pose


def follow_line():
    offline_readings = 0
    while(offline_readings < OFFLINE_LIMIT):
        print(offline_readings)
        if cs.reflected_light_intensity < THRESHOLD_EDGE:
            offline_readings = 0
            # Go but a little right
            robot.on(left_speed=MOTOR_HIGH, right_speed=MOTOR_LOW)
        else:
            offline_readings += 1
            # Go but a little left
            robot.on(left_speed=MOTOR_LOW, right_speed=MOTOR_HIGH)
    
    

def find_line():
    theta_current = 0
    i = 0
    while True:
        x_goal = [0, 10]
        y_goal = [0, 0]
        theta_goal = [90, 0]
        # turn(left_motor, right_motor, x_goal[i], y_goal[i], theta_goal[i], theta_current)        
        move_forward(left_motor, right_motor, x_goal[1], y_goal[1], theta_goal[1])
        break

def get_wheel_velocity(left_motor, right_motor):
    start_time = time()  # Record the starting time
    
    prev_left_encoder = left_motor.position
    prev_right_encoder = right_motor.position
    
    sleep(0.1)  # Sleep for a short time to allow the motors to move
    
    curr_left_encoder = left_motor.position
    curr_right_encoder = right_motor.position
    
    left_encoder_diff = curr_left_encoder - prev_left_encoder
    right_encoder_diff = curr_right_encoder - prev_right_encoder
    
    wheel_radius =  TIRE_DIAMETER / 2  # Replace with the actual radius of your wheel in centimeters
    time_interval = 0.1  # The time interval used for velocity calculation
    
    left_distance = (left_encoder_diff / 360) * 2 * PI * wheel_radius  # Convert encoder counts to distance
    right_distance = (right_encoder_diff / 360) * 2 * PI * wheel_radius  # Convert encoder counts to distance
    print("Left distance:", left_distance)
    

    end_time = time()  # Record the ending time
    delta_t = end_time - start_time  # Calculate the time difference
    
    left_velocity = left_distance / delta_t  # Calculate velocity in cm/s for left wheel
    right_velocity = right_distance / delta_t  # Calculate velocity in cm/s for right wheel
    print("Left velo:", left_velocity)
    
    return left_velocity, right_velocity, delta_t  # Return both velocities and delta_t


def turn(left_motor, right_motor, theta_goal, theta_current):
    while  theta_current - theta_goal > (3*PI/180):
        pass
        
    
     

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

def move_forward(left_motor, right_motor, x_goal, y_goal, theta_goal):
    global pose_past
    x_current = 0
    y_current = 0
    theta_current = 0

    while m.sqrt((x_goal-x_current)**2+(y_goal-y_current)**2) > 0.015: # and cs.reflected_light_intensity < THRESHOLD_SEARCH:
        x_current, y_current, theta_current = velocity_controller(left_motor, right_motor, x_goal, y_goal, theta_goal)
    return theta_current

def velocity_controller(left_motor, right_motor, x_goal, y_goal, theta_goal):
    global pose_past    
    #Pull velocity and time step values from encoders (Needs work)
    l_velo_current, r_velo_current, delta_t  = get_wheel_velocity(left_motor, right_motor)

    #Calc linear and angular velocity
    velo = (l_velo_current + r_velo_current)/2
    x_dot = m.cos(pose_past[2])*((l_velo_current + r_velo_current)/2)
    y_dot = m.sin(pose_past[2])*((l_velo_current + r_velo_current)/2)
    omega = (r_velo_current - l_velo_current)/BASE_WIDTH

    theta_current = pose_past[2] + omega*delta_t
    x_current = pose_past[0] + x_dot*delta_t
    y_current = pose_past[1] + y_dot*delta_t

    rho = m.sqrt((x_goal-x_current)**2+(y_goal-y_current)**2)
    alpha = m.atan2((y_goal-y_current),(x_goal-x_current))-theta_current
    beta = theta_goal-theta_current-alpha

    #Calculate new velocity for input into motor driver for next time step
    velo = K_RHO*rho
    omega = K_ALPHA*alpha + K_BETA*beta

    left_velocity = velo + omega*BASE_WIDTH
    right_velocity = velo - omega*BASE_WIDTH
    # print("Linear velocity:", velo)
    # print("x_dot:", x_dot)
    # print("y_dot:", y_dot)
    # print("Angular velocity:", omega)
    # print("Updated theta:", theta_current)
    # print("Updated x:", x_current)
    # print("Updated y:", y_current)
    # print("Rho:", rho)
    # print("Alpha:", alpha)
    # print("Beta:", beta)
    # print("New linear velocity command:", velo)
    # print("New angular velocity command:", omega)
    # print("Left wheel velocity:", left_velocity)
    # print("Right wheel velocity:", right_velocity)

    left_motor.on(speed=left_velocity)
    right_motor.on(speed=right_velocity)

    #Store previous pose for next loop
    pose_past = [x_current, y_current, theta_current]
    return pose_past

while not button.any():
    #follow_line()
    robot.off()
    find_line()
    robot.off()
    # exit()
# print(robot._gyro.angle_and_rate)
# robot.turn_degrees(speed=5, target_angle=1)
# while(1):
#     print(robot._gyro.angle_and_rate)