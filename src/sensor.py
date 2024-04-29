#!/usr/bin/env python3

# Sensor functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-06

from time import sleep
from math import cos, sin, radians, degrees, pi

# Sensor inputs
from ev3dev2.sensor import INPUT_3

# Sensor types
from ev3dev2.sensor.lego import UltrasonicSensor

from ev3dev2.motor import (
    MediumMotor,
    OUTPUT_C,
)

servo = MediumMotor(OUTPUT_C)
ultrasonic_sensor = UltrasonicSensor(address=INPUT_3)

# Six feet in cm
SIX_FEET = 182.88
# Limit for how far a line candidate can be from the expected position of the line
NEAREST_NEIGHBOUR_LIMIT = 30
# Offset from the servo motor to the ultrasonic sensor
RANGE_SCAN_OFFSET = 1.5

NUMBER_OF_POINTS_FOR_CERTAINTY = 1

MAX_WALL_GUESSES = 5

VALID_POINT_THRESHOLD = 50

WALL_CERTAINTY_RESOLUTION = 2

corners = [(0, 0), (0, SIX_FEET), (SIX_FEET, 0), (SIX_FEET, SIX_FEET)]
walls = [SIX_FEET, SIX_FEET, 0, 0]


def reset_walls_and_corners():
    global walls
    global corners
    walls = [SIX_FEET, SIX_FEET, 0, 0]
    corners = [(0, 0), (0, SIX_FEET), (SIX_FEET, 0), (SIX_FEET, SIX_FEET)]


def reset_servo():
    servo.reset()


def move_servo_to_angle(angle, speed=10, callback=lambda: sleep(0.1)):
    servo.on_to_position(speed, angle)
    while servo.is_running:
        callback()


def get_ultrasonic_distance():
    return ultrasonic_sensor.distance_centimeters


def transform_distance_to_coordinate(distance, angle, robot_pose):
    return [
        robot_pose[0] + (distance * (cos(radians(angle)))),
        robot_pose[1] + (distance * (sin(radians(angle)))),
    ]


def cardinal_direction_sensor_scan(width, resolution, robot_pose):
    # Init empty lists
    point_map = [[], [], [], []]
    # Loop through integers 0-3 and each cardinal direction
    for i in range(4):
        # Define start and end angles for each cardinal direction
        start = (i * 90) - width // 2
        end = (i * 90) + width // 2
        for angle in range(start, end, resolution):
            move_servo_to_angle(-angle + (robot_pose[2] * 180 / pi) - 90)
            distance = get_ultrasonic_distance()
            if distance > VALID_POINT_THRESHOLD:
                continue
            point = transform_distance_to_coordinate(
                distance - RANGE_SCAN_OFFSET, angle, robot_pose
            )
            point_map[i].append(point)
            sleep(0.2)
    return point_map


# Function to get vertical line based on mode of points
def get_vertical_line(points, line, resolution=1):
    # Convert x values of points to integers
    x = [round(point[0] / resolution) * resolution for point in points]
    # Get the mode of the x values
    line_locations = sorted(set(x), key=x.count)
    if len(line_locations) == 0:
        return None
    # Set limit based on line position
    if line == "L":
        limit = 0
    else:
        limit = SIX_FEET
    for i in range(min(MAX_WALL_GUESSES, len(line_locations))):
        line_location = line_locations[-i]
        line_location_points = x.count(line_location)
        if (
            line_location_points > NUMBER_OF_POINTS_FOR_CERTAINTY
            and abs(line_location - limit) < NEAREST_NEIGHBOUR_LIMIT
        ):
            return line_location + resolution / 2, line_location_points
    return None


# Function to get vertical line based on mode of points
def get_horizontal_line(points, line, resolution=1):
    # Convert y values of points to integers
    y = [round(point[1] / resolution) * resolution for point in points]
    # Get the mode of the y values
    line_locations = sorted(set(y), key=y.count)
    if len(line_locations):
        return None
    # Set limit based on line position
    if line == "U":
        limit = SIX_FEET
    else:
        limit = 0
    for i in range(min(MAX_WALL_GUESSES, len(line_locations))):
        line_location = line_locations[-i]
        line_location_points = y.count(line_location)
        if (
            line_location_points > NUMBER_OF_POINTS_FOR_CERTAINTY
            and abs(line_location - limit) < NEAREST_NEIGHBOUR_LIMIT
        ):
            return line_location + resolution / 2, line_location_points
    return None


# Function to identify walls based on four groups of points
def wall_identification(data, pose_past):

    # Split data into four groups
    data = [[(float(point[0]), float(point[1])) for point in group] for group in data]
    # Get vertical and horizontal lines candidates
    R_wall = get_vertical_line(data[0], "R", resolution=2)
    U_wall = get_horizontal_line(data[1], "U", resolution=2)
    L_wall = get_vertical_line(data[2], "L", resolution=2)
    D_wall = get_horizontal_line(data[3], "D", resolution=2)

    global walls
    global corners

    if R_wall is not None:
        walls[0] = R_wall[0]
    if U_wall is not None:
        walls[1] = U_wall[0]
    if L_wall is not None:
        walls[2] = L_wall[0]
    if D_wall is not None:
        walls[3] = D_wall[0]

    current_closest_corner = sorted(
        range(4),
        key=lambda i: (corners[i][0] - pose_past[0]) ** 2
        + (corners[i][1] - pose_past[1]) ** 2,
    )[0]

    prev_corner = corners[current_closest_corner]

    corners[0] = (walls[2], walls[3])
    corners[1] = (walls[2], walls[1])
    corners[2] = (walls[0], walls[3])
    corners[3] = (walls[0], walls[1])

    new_corner = corners[current_closest_corner]

    if new_corner != prev_corner:
        reset_walls_and_corners()

    map_file = open("map.csv", "a")
    map_file.write("C," + str(new_corner[0]) + "," + str(new_corner[1]) + "\n")
    map_file.close()

    return new_corner, prev_corner


# Test the sensor scan function
if __name__ == "__main__":
    servo.reset()
    # Clear map file
    map_file = open("map.csv", "w")
    map_file.write("")
    map_file.close()

    points_file = open("points.csv", "w")
    points_file.write("")
    points_file.close()

    cardinal_direction_sensor_scan(60, 5, (122.88, 122.88, 0))
