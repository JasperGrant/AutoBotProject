# Map Generation Functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-06

import matplotlib.pyplot as plt

SIX_FEET = 182.88


def get_vertical_line(points):
    x = [round(point[0]) for point in points]
    line_location = max(set(x), key=x.count)
    return [(line_location + 0.5, 0), (line_location + 0.5, SIX_FEET)]


def get_horizontal_line(points):
    y = [round(point[1]) for point in points]
    line_location = max(set(y), key=y.count)
    return [(0, line_location + 0.5), (SIX_FEET, line_location + 0.5)]


def wall_identification(data):
    walls = []
    even = True
    for group in split_into_cardinal_cones(data):
        print(group)
        group = [(float(point[0]), float(point[1])) for point in group[0]]
        if even:
            walls.append(get_vertical_line(group))
        else:
            walls.append(get_horizontal_line(group))
        even = not even

    return walls


def split_into_cardinal_cones(points):

    length = len(points)
    print(length)
    num_scans = length // 72
    if num_scans == 0:
        num_scans = 1
    up = []
    down = []
    left = []
    right = []
    for i in range(num_scans):
        direction = 72 // 4
        data = points[i * 72 : (i + 1) * 72]
        right.append(data[0:6] + data[-6:])
        up.append(data[direction - 6 : direction + 6])
        left.append(data[(2 * direction) - 6 : (2 * direction) + 6])
        down.append(data[(3 * direction) - 6 : (3 * direction) + 6])

    return [right, up, left, down]


map_file = open("map.txt", "r").read()

points = [
    [line.split(",")[0], line.split(",")[1]] for line in map_file.split("\n") if line
]

predicted_walls = wall_identification(points)

actual_walls = [
    [(0, 0), (0, SIX_FEET)],
    [(0, SIX_FEET), (SIX_FEET, SIX_FEET)],
    [(SIX_FEET, 0), (0, 0)],
    [(SIX_FEET, SIX_FEET), (SIX_FEET, 0)],
]

plt.figure()
for point in points:
    plt.scatter(float(point[0]), float(point[1]), color="red")
print(predicted_walls)
for wall in predicted_walls:
    plt.plot([point[0] for point in wall], [point[1] for point in wall], color="blue")
    print(wall)

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
