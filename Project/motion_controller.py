#!/usr/bin/env python3

import math as m
import EV3_math_modules as ev3_math
from time import sleep, time

PI = 3.14159

# Assuming we are on a line to start
# Number of readings for the robot to think it is going in the wrong direction
WRONG_DIRECTION_LIMIT = 10


NUM_DEGREES_FOR_EQUALITY = 3 * PI / 180
DISTANCE_FOR_EQUALITY = 2

# Velocity controller Gains
K_RHO = 0.7
K_ALPHA = 0.8
K_BETA = -0.5

# EV3 Parameters
BASE_WIDTH = 13  # cm
TIRE_DIAMETER = 5.8  # cm


def get_wheel_velocity(left_motor, right_motor, TIME):
    start_time = time()
    prev_left_encoder = left_motor.position
    prev_right_encoder = right_motor.position
    sleep(TIME)
    curr_left_encoder = left_motor.position
    curr_right_encoder = right_motor.position

    left_encoder_diff = curr_left_encoder - prev_left_encoder
    right_encoder_diff = curr_right_encoder - prev_right_encoder

    wheel_radius = TIRE_DIAMETER / 2

    left_distance = (left_encoder_diff / 360) * 2 * PI * wheel_radius
    right_distance = (right_encoder_diff / 360) * 2 * PI * wheel_radius

    end_time = time()
    delta_t = end_time - start_time

    left_velocity = left_distance / delta_t
    right_velocity = right_distance / delta_t

    print("Left wheel velocity:", left_velocity)
    print("Right wheel velocity:", right_velocity)
    print("Delta t:", delta_t)

    return left_velocity, right_velocity, delta_t


def velocity_controller(
    left_motor, right_motor, x_goal, y_goal, theta_goal, pose_past, is_turning=False
):
    # Pull velocity and time step values from encoders (Needs work)

    TIME = 0.5

    l_velo_current, r_velo_current, delta_t = get_wheel_velocity(
        left_motor, right_motor, TIME
    )

    # Calc linear and angular velocity
    x_dot = m.cos(pose_past[2]) * ((l_velo_current + r_velo_current) / 2)
    y_dot = m.sin(pose_past[2]) * ((l_velo_current + r_velo_current) / 2)
    omega = (r_velo_current - l_velo_current) / BASE_WIDTH

    theta_current = pose_past[2] + omega * delta_t
    x_current = pose_past[0] + x_dot * delta_t
    y_current = pose_past[1] + y_dot * delta_t

    rho = m.sqrt((x_goal - x_current) ** 2 + (y_goal - y_current) ** 2)
    alpha = (
        (m.atan2((y_goal - y_current), (x_goal - x_current)) - theta_current)
        if not is_turning
        else theta_goal - theta_current
    )
    beta = theta_goal - theta_current - alpha

    # Calculate new velocity for input into motor driver for next time step
    velo = K_RHO * rho
    omega = K_ALPHA * ev3_math.circle_minus(alpha) + K_BETA * ev3_math.circle_minus(
        beta
    )

    if is_turning:
        velo = 0
        omega = ev3_math.clamp(omega, 3 / BASE_WIDTH, 8 / BASE_WIDTH)
    else:
        velo = ev3_math.clamp(velo, 7, 12)
        omega = ev3_math.clamp(omega, 0, 15 / BASE_WIDTH)

    # if alpha > abs(PI/2):
    #     velo = 0

    left_velocity = velo - omega * BASE_WIDTH
    right_velocity = velo + omega * BASE_WIDTH
    # scale = abs(left_velocity/right_velocity)
    # if left_velocity < 20 and right_velocity < 20:
    #     speed_l=clamp(left_velocity,20, 70)
    #     speed_r=clamp(right_velocity,20, 70)/scale

    # print("Left wheel velocity:", left_velocity)
    # print("Right wheel velocity:", right_velocity)

    # print("Linear velocity:", velo_)
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

    # Store previous pose for next loop
    pose_past = [x_current, y_current, ev3_math.circle_minus(theta_current)]
    print("Pose:", pose_past)
    return pose_past
