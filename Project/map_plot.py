# Map Plot Generation Functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-06

import matplotlib.pyplot as plt
import seaborn as sns


def euclidean_distance(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5


def is_point_outlier(point, group):
    for points in group:
        for other_point in points:
            if point != other_point and euclidean_distance(point, other_point) < 10:
                return False
    return True


# Six feet in cm
SIX_FEET = 182.88

map_file = open("map9.csv", "r").read()
points = [point.split(",") for point in map_file.split("\n") if point]
# Change points into four groups by value of first char
group_of_points = [
    [[float(point[1]), float(point[2])] for point in points if point[0] == direction]
    for direction in "RULDCF"
]

robot_pose = [
    points.split(",") for points in open("pose.csv", "r").read().split("\n") if points
]
robot_pose = [[float(point[0]), float(point[1])] for point in robot_pose]

# Filter points without a point within 1 euclidean distance
group_of_points = [
    [point for point in group if not is_point_outlier(point, group_of_points)]
    for group in group_of_points
]
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
for point in group_of_points[5]:
    plt.scatter(float(point[0]), float(point[1]), color="blue")

for wall in actual_walls:
    plt.plot([point[0] for point in wall], [point[1] for point in wall], color="green")

for point in robot_pose:
    plt.plot(float(point[0]), float(point[1]), color="black")

plt.xlabel("X")
plt.ylabel("Y")
plt.title("Map of Environment")
# Make plot square
plt.gca().set_aspect("equal", adjustable="box")


# Second figure
# Plot seaborn heatmap of number of points on the map
plt.figure()
# Get the number of points in each grid 2cm grid square
grid = [[0 for _ in range(14)] for _ in range(14)]
for points in group_of_points:
    if points != group_of_points[4]:
        for point in points:
            grid[int(float(point[1]) / 15)][int(float(point[0]) / 15)] += 1

grid[11] = [20 for _ in range(14)]
grid[0] = [20 for _ in range(14)]
for row in grid:
    row[11] = 20
    row[0] = 20
# Plot the heatmap
ax = sns.heatmap(grid, cmap="magma", vmin=2, vmax=12)
# Display only rows 0 to 11 and columns 0 to 11
ax.invert_yaxis()
plt.xlim(0, 12)
plt.ylim(0, 12)
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Heatmap of Points in Environment")
plt.show()
