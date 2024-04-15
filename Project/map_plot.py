# Map Plot Generation Functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-06

import matplotlib.pyplot as plt

# Six feet in cm
SIX_FEET = 182.88
# Limit for how far a line candidate can be from the expected position of the line
NEAREST_NEIGHBOUR_LIMIT = 30
# Resolution for wall estimation
WALL_ESTIMATION_RESOLUTION = 5

# These will both be combined into one function at some point in the future


# Function to get vertical line based on mode of points
def get_vertical_line(points, line, resolution=1):
    # Convert x values of points to integers
    x = [round(point[0] / resolution) * resolution for point in points]
    # Get the mode of the x values
    line_locations = sorted(set(x), key=x.count)
    line_location = line_locations[-1]
    # Set limit based on line position
    if line == "L":
        limit = 0
    else:
        limit = SIX_FEET
    # If selected point is too far from nearest neighbour, remove it
    while abs(line_location - limit) > NEAREST_NEIGHBOUR_LIMIT:
        line_locations.pop()
        if line_locations == []:
            return [(-50, -50), (-50, -50)]
        line_location = line_locations[-1]
    # Return final line candidate as two points
    return [
        (line_location + resolution / 2, 0),
        (line_location + resolution / 2, SIX_FEET),
    ]


# Function to get vertical line based on mode of points
def get_horizontal_line(points, line, resolution=1):
    # Convert y values of points to integers
    y = [round(point[1] / resolution) * resolution for point in points]
    # Get the mode of the y values
    line_locations = sorted(set(y), key=y.count)
    line_location = line_locations[-1]
    # Set limit based on line position
    if line == "U":
        limit = SIX_FEET
    else:
        limit = 0
    # If selected point is too far from nearest neighbour, remove it
    while abs(line_location - limit) > NEAREST_NEIGHBOUR_LIMIT:
        line_locations.pop()
        if line_locations == []:
            return [(-50, -50), (-50, -50)]
        line_location = line_locations[-1]
    # Return final line candidate as two points
    return [
        (0, line_location + resolution / 2),
        (SIX_FEET, line_location + resolution / 2),
    ]


# Function to identify walls based on four groups of points
def wall_identification(data):
    walls = []
    # Split data into four groups
    data = [[(float(point[0]), float(point[1])) for point in group] for group in data]
    # Get vertical and horizontal lines for each group
    walls.append(get_vertical_line(data[0], "R", WALL_ESTIMATION_RESOLUTION))
    walls.append(get_horizontal_line(data[1], "U", WALL_ESTIMATION_RESOLUTION))
    walls.append(get_vertical_line(data[2], "L", WALL_ESTIMATION_RESOLUTION))
    # walls.append(get_horizontal_line(data[3], "D", WALL_ESTIMATION_RESOLUTION))

    return walls


map_file = open("map.csv", "r").read()
points = [point.split(",") for point in map_file.split("\n") if point]
# Change points into four groups by value of first char
group_of_points = [
    [[point[1], point[2]] for point in points if point[0] == direction]
    for direction in "RULDC"
]

predicted_walls = wall_identification(group_of_points)

actual_walls = [
    [(0, 0), (0, SIX_FEET)],
    [(0, SIX_FEET), (SIX_FEET, SIX_FEET)],
    [(SIX_FEET, 0), (0, 0)],
    [(SIX_FEET, SIX_FEET), (SIX_FEET, 0)],
]

# Plot the map
plt.figure()
for point in group_of_points[0]:
    plt.scatter(float(point[0]), float(point[1]), color="red")
for point in group_of_points[1]:
    plt.scatter(float(point[0]), float(point[1]), color="orange")
for point in group_of_points[2]:
    plt.scatter(float(point[0]), float(point[1]), color="yellow")
for point in group_of_points[3]:
    plt.scatter(float(point[0]), float(point[1]), color="pink")
for point in group_of_points[4]:
    plt.scatter(float(point[0]), float(point[1]), color="purple")

for wall in predicted_walls:
    plt.plot([point[0] for point in wall], [point[1] for point in wall], color="blue")

for wall in actual_walls:
    plt.plot([point[0] for point in wall], [point[1] for point in wall], color="green")

# Plot robot position
plt.scatter(60.96, 122.88, color="black")
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Map of Environment")
# Make plot square
plt.gca().set_aspect("equal", adjustable="box")
plt.show()
