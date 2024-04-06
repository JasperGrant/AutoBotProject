#!/usr/bin/env python3

PI = 3.14159


def circle_minus(angle):
    return (angle + PI) % (2 * PI) - PI


def clamp(value, min_value, max_value):
    if value < 0:
        return max(min(value, -min_value), -max_value)
    return max(min(value, max_value), min_value)
