# Map Plot Generation Functions for the Gerard SLAM project
# Written by Jasper Grant and Michael MacGillivray
# 2024-04-14

import matplotlib.pyplot as plt

# Open the file and read data
priority_file = open("behavior.txt", "r").read()
# Split the data into a list of step priorities
priorities = [priority.split(",") for priority in priority_file.split("\n") if priority]
# List behaviors
behaviors = ["Waypoint Follow", "Scan", "Obstacle Avoid"]
# Group the priorities by behavior in a dictionary
grouped_priorities = [
    {behaviors[i - 1]: [float(priority[i]) for priority in priorities]}
    for i in range(1, 4)
]
initial_time = float(priorities[0][0])
times = [float(priority[0]) - initial_time for priority in priorities]

# Plot the data
plt.figure()
for behavior in grouped_priorities:
    for key, value in behavior.items():
        plt.plot(times, value, label=key)
plt.legend()
plt.xlabel("Time")
plt.ylabel("Priority")
plt.title("Behavior Priorities")
plt.show()
