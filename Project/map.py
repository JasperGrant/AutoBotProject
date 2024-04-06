# Map Generation Functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-06

import matplotlib.pyplot as plt

map_file = open("map.txt", "r").read()

points = [
    [line.split(",")[0], line.split(",")[1]] for line in map_file.split("\n") if line
]
plt.figure()
for point in points:
    plt.scatter(float(point[0]), float(point[1]), color="red")
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Map of Environment")
plt.show()
