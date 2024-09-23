#!/usr/bin/env python3

import rospy
import rosbag
import sys
import argparse
import numpy
from tf2_ros import Buffer, TransformListener
from rosgraph_msgs.msg import Clock
import matplotlib.pyplot as plt
import time


class TransformHandler():
    def __init__(self, gt_frame, est_frame, max_time_between=0.01):
        self.gt_frame = gt_frame
        self.est_frame = est_frame
        self.frames = [gt_frame, est_frame]

        self.tf_buffer = Buffer(cache_time=rospy.Duration(max_time_between))
        self.__tf_listener = TransformListener(self.tf_buffer)

    def get_transform(self, fixed_frame, target_frame):
        # caller should handle the exceptions
        return self.tf_buffer.lookup_transform(target_frame, fixed_frame, rospy.Time(0))


def get_errors(transform):
    tr = transform.transform.translation
    return numpy.linalg.norm([tr.x, tr.y])


# Argument parsing
parser = argparse.ArgumentParser()
parser.add_argument(
    '--gt_frame', help='The child frame of the GT transform', default='mocap_laser_link')
parser.add_argument(
    '--est_frame', help='The child frame of the estimation transform', default='base_scan')

args = parser.parse_args()
gt_frame = args.gt_frame
est_frame = args.est_frame

# ROS node initialization
rospy.init_node('evaluation_node')

if rospy.rostime.is_wallclock():
    rospy.logfatal(
        'You should be using simulated time: rosparam set use_sim_time true')
    sys.exit(1)

rospy.loginfo('Waiting for clock')
rospy.sleep(0.00001)

handler = TransformHandler(gt_frame, est_frame, max_time_between=20)  # 500ms

# Variables to store error values and time
error_values = []
start_time = time.time()
duration = 30  # Run for 30 seconds
sampling_rate = 1000  # Hz

rospy.loginfo('Listening to frames and computing error, press Ctrl-C to stop')

sleeper = rospy.Rate(sampling_rate)
try:
    while not rospy.is_shutdown():
        current_time = time.time()
        elapsed_time = current_time - start_time

        if elapsed_time > duration:
            rospy.loginfo("30 seconds passed. Stopping error collection.")
            break

        try:
            t = handler.get_transform(gt_frame, est_frame)
        except Exception as e:
            rospy.logwarn(e)
            error_values.append(None)  # Append None if an error occurs
        else:
            eucl = get_errors(t)
            error_values.append(eucl * 1e3)  # Convert to mm and store error

        try:
            sleeper.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            rospy.logwarn(e)

except rospy.exceptions.ROSInterruptException:
    pass

# After 30 seconds, plot the error values
rospy.loginfo("Generating the error graph...")

# Clean up None values from the list
error_values = [x for x in error_values if x is not None]

# Create a time vector
time_values = numpy.linspace(0, duration, len(error_values))

# Plot the errors
plt.figure()
plt.plot(time_values, error_values, label='Error (mm)')
plt.xlabel('Time (seconds)')
plt.ylabel('Error (mm)')
plt.title('Error over 30 seconds')
plt.legend()
plt.grid(True)

# Save and show the plot
plt.savefig('error_plot.png')
plt.show()
