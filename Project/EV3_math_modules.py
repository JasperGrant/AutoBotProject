#!/usr/bin/env python3

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
