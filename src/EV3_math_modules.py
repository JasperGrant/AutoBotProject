#!/usr/bin/env python3


# Math functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-13


PI = 3.14159


def circle_minus(angle):
    return (angle + PI) % (2 * PI) - PI


def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    elif value > max_value:
        return max_value
    else:
        return value


def matrix_addition(matrix1, matrix2):
    result = []
    for i in range(len(matrix1)):
        result.append([])
        for j in range(len(matrix1[0])):
            result[i].append(matrix1[i][j] + matrix2[i][j])
    return result


def zeros_matrix(rows, cols):
    """
    Creates a matrix filled with zeros.
        :param rows: the number of rows the matrix should have
        :param cols: the number of columns the matrix should have

        :returns: list of lists that form the matrix.
    """
    M = []
    while len(M) < rows:
        M.append([])
        while len(M[-1]) < cols:
            M[-1].append(0.0)

    return M


def matrix_multiply(A, B):
    """
    Returns the product of the matrix A * B
        :param A: The first matrix - ORDER MATTERS!
        :param B: The second matrix

        :return: The product of the two matrices
    """
    rowsA = len(A)
    colsA = len(A[0])

    rowsB = len(B)
    colsB = len(B[0])

    if colsA != rowsB:
        raise ArithmeticError("Number of A columns must equal number of B rows.")

    C = zeros_matrix(rowsA, colsB)

    for i in range(rowsA):
        for j in range(colsB):
            total = 0
            for ii in range(colsA):
                total += A[i][ii] * B[ii][j]
            C[i][j] = total

    return C


def matrix_multiplication(matrix1, matrix2):
    # Check whether matrices can be multiplied
    assert len(matrix1[0]) == len(matrix2)
    result = []
    for i in range(len(matrix1)):
        result.append([])
        for j in range(len(matrix2[0])):
            result[i].append(0)
            for k in range(len(matrix2)):
                result[i][j] += matrix1[i][k] * matrix2[k][j]
    return result


def matrix_transpose(matrix):
    result = []
    for i in range(len(matrix[0])):
        result.append([])
        for j in range(len(matrix)):
            result[i].append(matrix[j][i])
    return result


def matrix_inversion(matrix):
    det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]
    assert det != 0
    return [
        [matrix[1][1] / det, -matrix[0][1] / det],
        [-matrix[1][0] / det, matrix[0][0] / det],
    ]


def transformation_matrix(x, y, theta_rad, x_prime, y_prime):
    # Create the homogeneous transformation matrix
    homogenous_matrix = np.array(
        [
            [np.cos(theta_rad), -np.sin(theta_rad), x],
            [np.sin(theta_rad), np.cos(theta_rad), y],
            [0, 0, 1],
        ]
    )

    # Create the vector of coordinates
    vector = np.array([[x_prime], [y_prime], [1]])

    # Multiply the homogeneous transformation matrix by the vector of coordinates
    transformed_vector = homogenous_matrix @ vector

    return transformed_vector[:2]
