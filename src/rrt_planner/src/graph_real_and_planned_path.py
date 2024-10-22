import rosbag
import matplotlib.pyplot as plt

bag_path = '../results/2024-10-17-17-28-24.bag'

amcl_x, amcl_y = [], []
global_plan_segments = []
current_global_plan_x, current_global_plan_y = [], []

global_plan = '/move_base/RRTPlannerROS/global_plan'

with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/amcl_pose', global_plan]):
        if topic == '/amcl_pose':
            amcl_x.append(msg.pose.pose.position.x)
            amcl_y.append(msg.pose.pose.position.y)
        elif topic == global_plan:
            if current_global_plan_x:
                global_plan_segments.append(
                    (current_global_plan_x, current_global_plan_y))
                current_global_plan_x, current_global_plan_y = [], []

            for pose in msg.poses:
                current_global_plan_x.append(pose.pose.position.x)
                current_global_plan_y.append(pose.pose.position.y)

if current_global_plan_x:
    global_plan_segments.append((current_global_plan_x, current_global_plan_y))

plt.plot(amcl_x, amcl_y, label='Actual Path', color='blue')

plt.plot(global_plan_segments[0][0], global_plan_segments[0][1],
         linestyle='--', color='red')
plt.scatter(global_plan_segments[0][0][0], global_plan_segments[0][1][0],
            color='green', marker='o', s=50, label='Planned Start')
plt.scatter(global_plan_segments[0][0][-1], global_plan_segments[0][1][-1],
            color='orange', marker='x', s=50, label='Planned End')

for i, (global_plan_x, global_plan_y) in enumerate(global_plan_segments[:-32]):

    plt.plot(global_plan_x, global_plan_y,
             linestyle='--', color='red')
    plt.scatter(global_plan_x[0], global_plan_y[0],
                color='green', marker='o', s=50)
    plt.scatter(global_plan_x[-1], global_plan_y[-1],
                color='orange', marker='x', s=50)

plt.scatter(amcl_x[0], amcl_y[0], color='purple',
            marker='o', s=150, label='Start')
plt.scatter(amcl_x[-1], amcl_y[-1], color='black',
            marker='*', s=200, label='End')

plt.xlabel('Position X[m]')
plt.ylabel('Position Y[m]')
plt.title('Actual vs Planned Path with Multiple Goals')

plt.legend()
plt.savefig('../results/actual_vs_planned_path.png')
plt.show()
