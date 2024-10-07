#!/usr/bin/env python3

import rospy
import rosbag
import sys
import argparse
import numpy
import csv
from tf2_ros import Buffer, TransformListener
from rosgraph_msgs.msg import Clock
import time
import tf


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
    # Calculate the error for X and Y axes only
    translation_error_x = tr.x
    translation_error_y = tr.y

    # Calculate the orientation error using quaternions (for Yaw only)
    rot = transform.transform.rotation
    orientation_quat = [rot.x, rot.y, rot.z, rot.w]
    # Convert to roll, pitch, yaw (we only care about yaw here)
    _, _, yaw = tf.transformations.euler_from_quaternion(orientation_quat)

    return translation_error_x, translation_error_y, yaw


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
duration = 60  # Run for 60 seconds
sampling_rate = 1000  # Hz

rospy.loginfo('Listening to frames and computing error, press Ctrl-C to stop')

sleeper = rospy.Rate(sampling_rate)
try:
    while not rospy.is_shutdown():
        current_time = time.time()
        elapsed_time = current_time - start_time

        if elapsed_time > duration:
            rospy.loginfo("60 seconds passed. Stopping error collection.")
            break

        try:
            t = handler.get_transform(gt_frame, est_frame)
        except Exception as e:
            rospy.logwarn(e)
            # Append None if an error occurs
            error_values.append((elapsed_time, None, None, None))
        else:
            translation_error_x, translation_error_y, yaw = get_errors(t)
            # Store error in X, Y, and Yaw
            error_values.append(
                (elapsed_time, translation_error_x * 1e3, translation_error_y * 1e3, yaw))

        try:
            sleeper.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            rospy.logwarn(e)

except rospy.exceptions.ROSInterruptException:
    pass

# After 60 seconds, save the error values to a CSV file
rospy.loginfo("Saving error values to a CSV file...")

cleaned_error_values = [row for row in error_values if row[1] is not None]

# Write the cleaned data to a CSV file
csv_file = 'error_data.csv'
csv_file = 'error_data.csv'
with open(csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time (seconds)", "Error X (mm)",
                    "Error Y (mm)", "Yaw (rad)"])  # Updated header row
    writer.writerows(cleaned_error_values)

rospy.loginfo(f"Data saved to {csv_file}")