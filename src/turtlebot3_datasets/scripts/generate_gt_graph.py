import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv('../results/odometry_data.csv')

print(data.columns)

x = data['field.pose.pose.position.x'].to_numpy()
y = data['field.pose.pose.position.y'].to_numpy()

plt.figure(figsize=(8, 8))
plt.plot(x, y, label='Robot Path (Odometry)', color='blue')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Ground Truth Path from Odometry')
plt.legend()
plt.grid(True)

plt.savefig('../results/ground_truth_path.png')
