#!/usr/bin/env python3

# EKF for the Gerard SLAM project
# Jasper Grant and Michael MacGillivray
# 2024-04-05

from math import pi, sin, cos, atan2, sqrt
import EV3_math_modules as ev3_math
from time import sleep, time


# called from motor_control.py
# Updateing covariavce between scans, stae is updated in the motor control code.
def propagate_state_covariance(
    velocitiy, time, state, state_covariance=[[0, 0, 0], [0, 0, 0], [0, 0, 0]]
):
    # Need velosity setpoint, time, and previous state
    # Q is the process noise covariance matrix
    Q = [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]]

    # Calculate the Jacobian of the state transition matrix
    G = [
        [1, 0, -velocitiy * sin(state[2]) * time],
        [0, 1, velocitiy * cos(state[2]) * time],
        [0, 0, 1],
    ]

    # Calculate the predicted state covariance
    pred_covariance = ev3_math.matrix_addition(
        ev3_math.matrix_multiplication(
            ev3_math.matrix_multiplication(G, state_covariance),
            ev3_math.matrix_transpose(G),
        ),
        Q,
    )
    return pred_covariance


def update_state(landmark, pred_covariance, state):
    # Need landmark measurements and previous state
    # R is the measurement noise covariance matrix
    R = [[0.01, 0.0], [0.0, 0.01]]

    # Calculate the Jacobian of the measurement model
    H = [[1, 0, 0], [0, 1, 0]]

    # Calculate the Kalman gain
    K = ev3_math.matrix_multiplication(
        ev3_math.matrix_multiplication(pred_covariance, ev3_math.matrix_transpose(H)),
        ev3_math.matrix_inversion(
            ev3_math.matrix_addition(
                ev3_math.matrix_multiplication(
                    ev3_math.matrix_multiplication(H, pred_covariance),
                    ev3_math.matrix_transpose(H),
                ),
                R,
            )
        ),
    )

    # Calculate the innovation
    z = [[landmark[0] - state[0]], [landmark[1] - state[1]]]

    # Update the state
    state = ev3_math.matrix_addition(state, ev3_math.matrix_multiplication(K, z))

    # Update the state covariance
    state_covariance = ev3_math.matrix_multiplication(
        ev3_math.matrix_addition(
            [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            ev3_math.matrix_multiplication(
                ev3_math.matrix_multiplication(-K, H), state
            ),
        ),
        pred_covariance,
    )

    return state, state_covariance
