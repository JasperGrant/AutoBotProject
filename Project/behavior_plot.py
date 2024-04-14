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
    {behaviors[i]: [float(priority[i]) for priority in priorities]} for i in range(3)
]

# Plotting the priority of each behavior each step
for behavior in grouped_priorities:
    for key in behavior:
        plt.plot(behavior[key], label=key)
plt.xlabel("Step")
plt.xticks(range(0, len(priorities), 1))
plt.ylabel("Priority")
plt.title("Behavior Priorities Each Step")
plt.legend()
plt.show()
