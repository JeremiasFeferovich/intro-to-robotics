import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load the CSV file
data = pd.read_csv('../results/odometry_data.csv')

# Access the correct columns
x = data['field.pose.pose.position.x'].values
y = data['field.pose.pose.position.y'].values

# Plot the robot's path
plt.plot(x, y, color='blue')

# Mark the start point
plt.scatter(x[0], y[0], color='g', marker='o', s=100, label='Start')

# Define the goal positions from the bag file (map frame)
goal_positions = [
    {'x': 1.07, 'y': -1.96, 'label': 'Goal 1'},
    {'x': 4.92, 'y': -4.45, 'label': 'Goal 2'},
    {'x': 5.29, 'y': -0.61, 'label': 'Goal 3'},
    {'x': 2.89, 'y': -1.56, 'label': 'Goal 4'}
]

for goal in goal_positions:
    plt.scatter(goal['x'], goal['y'], label=goal['label'], marker='*', s=100)

# Set plot limits based on the robot's path to ensure visibility of all points
plt.xlim(min(x) - 1, max(x) + 1)
plt.ylim(min(y) - 1, max(y) + 1)

# Add labels and title
plt.xlabel('Position X')
plt.ylabel('Position Y')
plt.title('Robot Pose (X vs Y) with Goals')

plt.legend()

plt.savefig('../results/robot_path_with_goals.png')
