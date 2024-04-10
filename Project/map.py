# Map Generation Functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-06

import matplotlib.pyplot as plt
import random


def get_random_points(data, n):
    # Return n random consecutive points from data
    start = random.randint(0, len(data) - n)
    return data[start : start + n]


def get_best_fit_line(points):
    x = [float(point[0]) for point in points]
    y = [float(point[1]) for point in points]
    x_mean = sum(x) / len(x)
    y_mean = sum(y) / len(y)
    x_diff = [x[i] - x_mean for i in range(len(x))]
    y_diff = [y[i] - y_mean for i in range(len(y))]
    slope = sum([x_diff[i] * y_diff[i] for i in range(len(x))]) / sum(
        [x_diff[i] ** 2 for i in range(len(x))]
    )
    y_intercept = y_mean - slope * x_mean
    return slope, y_intercept


def get_distance_from_line(point, line_slope, line_intercept):
    return abs(line_slope * point[0] - point[1] + line_intercept) / (
        (line_slope**2 + 1) ** 0.5
    )


def RANSAC(num_inliers, max_iterations, data, fit_threshold, num_close_points_required):
    walls = []
    for _ in range(max_iterations):
        inliers = get_random_points(data, num_inliers)
        line_slope, line_intercept = get_best_fit_line(inliers)
        inliers = [
            point
            for point in data
            if get_distance_from_line(point, line_slope, line_intercept) < fit_threshold
        ]
        if len(inliers) > num_close_points_required:
            line_slope, line_intercept = get_best_fit_line(inliers)
            # Cut off the wall at the ends of the inliers
            x_min = min([point[0] for point in inliers])
            x_max = max([point[0] for point in inliers])
            y_min = line_slope * x_min + line_intercept
            y_max = line_slope * x_max + line_intercept
            walls.append([(x_min, y_min), (x_max, y_max)])
    # Return list of walls represented as two points each
    return walls


def get_vertical_line(points):
    x = [round(point[0]) for point in points]
    line_location = max(set(x), key=x.count)
    return [(line_location, 0), (line_location, 182.88)]


def get_horizontal_line(points):
    y = [round(point[1]) for point in points]
    line_location = max(set(y), key=y.count)
    return [(0, line_location), (182.88, line_location)]


def wall_identification(data):
    walls = []
    even = True
    for group in [
        data[64:72] + data[0:6],
        data[10:24],
        data[28:42],
        data[46:60],
    ]:
        group = [(float(point[0]), float(point[1])) for point in group]
        if even:
            walls.append(get_vertical_line(group))
        else:
            walls.append(get_horizontal_line(group))
        even = not even

    return walls


map_file = open("map.txt", "r").read()

points = [
    [line.split(",")[0], line.split(",")[1]] for line in map_file.split("\n") if line
]

corner_points = points[6:10] + points[24:28] + points[42:46] + points[60:64]

predicted_walls = wall_identification(points)
actual_walls = [
    [(0, 0), (0, 182.88)],
    [(0, 182.88), (182.88, 182.88)],
    [(182.88, 0), (0, 0)],
    [(182.88, 182.88), (182.88, 0)],
]
plt.figure()
for point in points:
    plt.scatter(float(point[0]), float(point[1]), color="red")
for point in corner_points:
    plt.scatter(float(point[0]), float(point[1]), color="orange")
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
