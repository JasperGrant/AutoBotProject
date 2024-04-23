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

# Initialize global point map
point_map = [[], [], [], [], []]

# Six feet in cm
SIX_FEET = 182.88
# Limit for how far a line candidate can be from the expected position of the line
NEAREST_NEIGHBOUR_LIMIT = 10
# Offset from the servo motor to the ultrasonic sensor
RANGE_SCAN_OFFSET = 1.5


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


def sensor_scan(width, resolution, robot_pose):
    points = []
    for angle in range(0, -width, -resolution):
        distances = []
        move_servo_to_angle(angle)
        for _ in range(3):  # Take three measurements
            distance = get_ultrasonic_distance()
            if distance != 255:
                distances.append(distance)
                points_file = open("points.csv", "a")
                points_file.write(str(point[0]) + " , " + str(point[1]) + "\n")
                points_file.close()
        if distances:
            averaged_distance = sum(distances) / len(distances)
            map_file = open("map.csv", "a")
            point = transform_distance_to_coordinate(
                averaged_distance, -angle, robot_pose
            )
            points.append(point)
            map_file.write(str(point[0]) + " , " + str(point[1]) + "\n")
            map_file.close()
        sleep(0.3)  # Wait before moving to the next angle
    return points


def cardinal_direction_sensor_scan(width, resolution, robot_pose):
    # Init empty lists
    global point_map
    # Cardinal directions Right, Up, Left, Down
    cardinal_directions = ["R", "U", "L", "D"]
    # Loop through integers 0-3 and each cardinal direction
    for i, direction in enumerate(cardinal_directions):
        # Define start and end angles for each cardinal direction
        start = (i * 90) - width // 2
        end = (i * 90) + width // 2
        for angle in range(start, end, resolution):
            distances = []
            move_servo_to_angle(-angle + (robot_pose[2] * 180 / pi) - 90)
            distance = get_ultrasonic_distance()
            if distance > 75:
                continue
            for _ in range(5):  # Take three measurements
                distance = get_ultrasonic_distance()
                distances.append(distance)
                points_file = open("points.csv", "a")
                points_file.write(str(distance))
                points_file.close()
            averaged_distance = sum(distances) / len(distances)
            map_file = open("map.csv", "a")
            point = transform_distance_to_coordinate(
                averaged_distance - RANGE_SCAN_OFFSET, angle, robot_pose
            )
            point_map[i].append(point)
            map_file.write(direction + "," + str(point[0]) + "," + str(point[1]) + "\n")
            map_file.close()
            sleep(0.2)


# Function to get vertical line based on mode of points
def get_vertical_line(points, line):
    # Convert x values of points to integers
    x = [round(point[0]) for point in points]
    # Get the mode of the x values
    line_locations = sorted(set(x), key=x.count)
    if line_locations == []:
        return [(-50, -50), (-50, -50)]
    line_location = line_locations[-1]
    # # Set limit based on line position
    # if line == "L":
    #     limit = 0
    # else:
    #     limit = SIX_FEET
    # # If selected point is too far from nearest neighbour, remove it
    # while abs(line_location - limit) > NEAREST_NEIGHBOUR_LIMIT:
    #     line_locations.pop()
    #     if line_locations == []:
    #         return [(0, 0), (0, 0)]
    #     line_location = line_locations[-1]
    # # Return final line candidate as two points
    return [(line_location + 0.5, 0), (line_location + 0.5, SIX_FEET)]


# Function to get vertical line based on mode of points
def get_horizontal_line(points, line):
    # Convert y values of points to integers
    y = [round(point[1]) for point in points]
    # Get the mode of the y values
    line_locations = sorted(set(y), key=y.count)
    if line_locations == []:
        return [(-50, -50), (-50, -50)]
    line_location = line_locations[-1]
    # # Set limit based on line position
    # if line == "U":
    #     limit = SIX_FEET
    # else:
    #     limit = 0
    # # If selected point is too far from nearest neighbour, remove it
    # while abs(line_location - limit) > NEAREST_NEIGHBOUR_LIMIT:
    #     line_locations.pop()
    #     if line_locations == []:
    #         return [(-50, -50), (-50, -50)]
    #     line_location = line_locations[-1]
    # # Return final line candidate as two points
    return [(0, line_location + 0.5), (SIX_FEET, line_location + 0.5)]


# Function to identify walls based on four groups of points
def wall_identification(data):
    global point_map
    walls = []
    # Split data into four groups
    data = [
        [(float(point[0]), float(point[1])) for point in group] for group in data[0:-1]
    ]
    # Get vertical and horizontal lines for each group
    walls.append(get_vertical_line(data[0], "R"))
    walls.append(get_horizontal_line(data[1], "U"))
    walls.append(get_vertical_line(data[2], "L"))
    walls.append(get_horizontal_line(data[3], "D"))
    map_file = open("map.csv", "a")

    corners = [
        (walls[0][0][0], walls[1][0][1]),
        (walls[0][0][0], walls[3][0][1]),
        (walls[2][0][0], walls[3][0][1]),
        (walls[2][0][0], walls[1][0][1]),
    ]
    point_map[4] = corners
    assert len(corners) == 4
    for point in corners:
        map_file.write("C," + str(point[0]) + "," + str(point[1]) + "\n")
    map_file.close()

    return walls


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
