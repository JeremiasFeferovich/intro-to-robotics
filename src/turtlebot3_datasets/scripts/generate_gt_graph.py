import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
data = pd.read_csv('../results/odometry_data.csv')

# Print column names for verification
print(data.columns)

# Extract the x and y coordinates
x = data['field.pose.pose.position.x'].to_numpy()
y = data['field.pose.pose.position.y'].to_numpy()

# Plot the path
plt.figure(figsize=(8, 8))
plt.plot(x, y, label='Robot Path (Odometry)', color='blue')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Ground Truth Path from Odometry')
plt.legend()
plt.grid(True)

# Show the plot
plt.savefig('../results/ground_truth_path.png')
