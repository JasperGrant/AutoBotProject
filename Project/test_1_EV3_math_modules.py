import pytest

from EV3_math_modules import (
    circle_minus,
    clamp,
    matrix_addition,
    matrix_multiplication,
    matrix_transpose,
    matrix_inversion,
)

from math import pi


@pytest.mark.parametrize(
    "angle,expected",
    [
        (0, 0),
        (pi, -pi),
        (-pi, pi),
        (2 * pi, 0),
        (-2 * pi, 0),
        (3 * pi, -pi),
        (-3 * pi, pi),
    ],
)
def test_circle_minus(angle, expected):
    assert abs(circle_minus(angle) - expected) < 0.0001


@pytest.mark.parametrize(
    "value,min_value,max_value,expected",
    [
        (30, 5, 60, 30),
        (-30, -5, 60, -5),
        (30, 5, 20, 20),
        (30, 20, 40, 30),
        (-30, -60, -20, -30),
        (-30, -60, -40, -40),
        (-10, -60, -20, -20),
    ],
)
def test_clamp(value, min_value, max_value, expected):
    assert clamp(value, min_value, max_value) == expected


@pytest.mark.parametrize(
    "matrix1,matrix2,expected",
    [
        ([[1, 2], [3, 4]], [[1, 2], [3, 4]], [[2, 4], [6, 8]]),  # 2x2 matrix
        (
            [[1, 2, 3], [4, 5, 6]],
            [[1, 2, 3], [4, 5, 6]],
            [[2, 4, 6], [8, 10, 12]],
        ),  # 2x3 matrix
        (
            [[1, 2], [3, 4], [5, 6]],
            [[1, 2], [3, 4], [5, 6]],
            [[2, 4], [6, 8], [10, 12]],
        ),  # 3x2 matrix
        (
            [[1, 2, 3], [4, 5, 6], [7, 8, 9]],
            [[1, 2, 3], [4, 5, 6], [7, 8, 9]],
            [[2, 4, 6], [8, 10, 12], [14, 16, 18]],
        ),  # 3x3 matrix
        ([[1]], [[1]], [[2]]),  # 1x1 matrix
        ([[1, 2]], [[1, 2]], [[2, 4]]),  # 1x2 matrix
        ([[1], [2]], [[1], [2]], [[2], [4]]),  # 2x1 matrix
    ],
)
def test_matrix_addition(matrix1, matrix2, expected):
    assert matrix_addition(matrix1, matrix2) == expected


@pytest.mark.parametrize(
    "matrix1,matrix2,expected",
    [
        ([[1, 2], [3, 4]], [[1, 2], [3, 4]], [[7, 10], [15, 22]]),  # 2x2 and 1x1 matrix
        ([[1]], [[1]], [[1]]),  # 1x1 and 1x1 matrix
        ([[1, 2], [3, 4]], [[1], [2]], [[5], [11]]),  # 2x2 and 2x1 matrix
        ([[1], [2]], [[1, 2]], [[1, 2], [2, 4]]),  # 2x1 and 1x2 matrix
        (
            [[1, 2, 3], [4, 5, 6]],
            [[1, 2], [3, 4], [5, 6]],
            [[22, 28], [49, 64]],
        ),  # 2x3 and 3x2 matrix
        (
            [[1, 2, 3], [4, 5, 6], [7, 8, 9]],
            [[1, 2, 3], [4, 5, 6], [7, 8, 9]],
            [[30, 36, 42], [66, 81, 96], [102, 126, 150]],
        ),  # 3x3 and 3x3 matrix
    ],
)
def test_matrix_multiplication(matrix1, matrix2, expected):
    assert matrix_multiplication(matrix1, matrix2) == expected


def test_matrix_multiplication_exception():
    with pytest.raises(AssertionError):
        matrix_multiplication([[1, 2], [3, 4]], [[1, 2], [3, 4], [5, 6]])


@pytest.mark.parametrize(
    "matrix,expected",
    [
        (  # 2x2 matrix
            [[1, 2], [3, 4]],
            [[1, 3], [2, 4]],
        ),
        (  # 3x2 matrix
            [[1, 2], [3, 4], [5, 6]],
            [[1, 3, 5], [2, 4, 6]],
        ),
        (  # 2x3 matrix
            [[1, 2, 3], [4, 5, 6]],
            [[1, 4], [2, 5], [3, 6]],
        ),
        (  # 3x3 matrix
            [[1, 2, 3], [4, 5, 6], [7, 8, 9]],
            [[1, 4, 7], [2, 5, 8], [3, 6, 9]],
        ),
        (  # 1x1 matrix
            [[1]],
            [[1]],
        ),
        (  # 1x2 matrix
            [[1, 2]],
            [[1], [2]],
        ),
        (  # 2x1 matrix
            [[1], [2]],
            [[1, 2]],
        ),
    ],
)
def test_matrix_transpose(matrix, expected):
    assert matrix_transpose(matrix) == expected


@pytest.mark.parametrize(
    "matrix,expected",
    [
        # Only 2x2 tests
        (
            [[1, 2], [3, 4]],
            [[-2, 1], [1.5, -0.5]],
        ),
        ([[1, 0], [0, 1]], [[1, 0], [0, 1]]),
        ([[1, 2], [3, 4]], [[-2, 1], [1.5, -0.5]]),
        ([[1, 3], [2, 4]], [[-2, 1.5], [1, -0.5]]),
    ],
)
def test_matrix_inversion(matrix, expected):
    assert matrix_inversion(matrix) == expected


@pytest.mark.parametrize(
    "matrix,expected",
    [
        ([[1, 2], [2, 4]], [[-2, 1], [1, -0.5]]),
        ([[1, 2], [3, 6]], [[-3, 1], [2, -0.5]]),
        ([[1, 3], [2, 6]], [[-6, 3], [2, -1]]),
    ],
)
def test_matrix_inversion_exception(matrix, expected):
    with pytest.raises(AssertionError):
        matrix_inversion(matrix)
