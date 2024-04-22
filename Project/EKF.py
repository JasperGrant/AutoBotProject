#!/usr/bin/env python3

# EKF for the Gerard SLAM project
# Jasper Grant and Michael MacGillivray
# 2024-04-05

from math import pi, sin, cos, atan2, sqrt
import LinearAlgebraPurePython as la
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
    pred_covariance = la.matrix_addition(
        la.matrix_multiply(
            la.matrix_multiply(G, state_covariance),
            la.matrix_multiply(G),
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
    K = la.matrix_multiply(
        la.matrix_multiply(pred_covariance, la.matrix_multiply(H)),
        la.invert_matrix(
            la.matrix_addition(
                la.matrix_multiply(
                    la.matrix_multiply(H, pred_covariance),
                    la.transpose(H),
                ),
                R,
            ),
            1,
        ),
    )

    # Calculate the innovation
    z = [[landmark[0] - state[0]], [landmark[1] - state[1]]]

    # Update the state
    state = la.matrix_addition(state, la.matrix_multiply(K, z))

    # Update the state covariance
    state_covariance = la.matrix_multiply(
        la.matrix_addition(
            [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            la.matrix_multiply(la.matrix_multiply(-K, H), state),
        ),
        pred_covariance,
    )

    return state, state_covariance
