#!/usr/bin/env python3

from math import atan2, sqrt
import EV3_math_modules as ev3_math

from odometry import BASE_WIDTH, get_pose_past


# Velocity controller Gains
K_RHO = 0.7
K_ALPHA = 0.6
K_BETA = -0.5

# EV3 Parameters
BASE_WIDTH = 13  # cm


def velocity_controller(
    left_motor, right_motor, x_goal, y_goal, theta_goal, is_turning=False
):
    # Pull velocity and time step values from encoders (Needs work)

    pose_past = get_pose_past()

    rho = sqrt((x_goal - pose_past[0]) ** 2 + (y_goal - pose_past[1]) ** 2)
    alpha = (
        (atan2((y_goal - pose_past[1]), (x_goal - pose_past[0])) - pose_past[2])
        if not is_turning
        else theta_goal - pose_past[2]
    )
    beta = theta_goal - pose_past[2] - alpha

    # Calculate new velocity for input into motor driver for next time step
    velo = K_RHO * rho
    omega = K_ALPHA * ev3_math.circle_minus(alpha) + K_BETA * ev3_math.circle_minus(
        beta
    )

    if is_turning:
        velo = 0
        if omega < 0:
            omega = ev3_math.clamp(omega, -8 / BASE_WIDTH, -3 / BASE_WIDTH)
        else:
            omega = ev3_math.clamp(omega, 3 / BASE_WIDTH, 8 / BASE_WIDTH)
    else:
        velo = ev3_math.clamp(velo, 7, 12)
        omega = ev3_math.clamp(omega, -5 / BASE_WIDTH, 5 / BASE_WIDTH)

    # if alpha > abs(pi/2):
    #     velo = 0

    left_velocity = velo - omega * BASE_WIDTH
    right_velocity = velo + omega * BASE_WIDTH

    left_motor.on(speed=left_velocity)
    right_motor.on(speed=right_velocity)
