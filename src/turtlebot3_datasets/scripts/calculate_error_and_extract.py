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

# Variables to store error values, GT path, and time
error_values = []
gt_path = []
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
            gt_transform = handler.get_transform(gt_frame, est_frame)
        except Exception as e:
            rospy.logwarn(f"Failed to get GT transform: {e}")
            # Append None for GT path if an error occurs
            gt_path.append((elapsed_time, None, None, None))
        else:
            # Get the ground truth X, Y, and Yaw
            gt_translation_error_x, gt_translation_error_y, gt_yaw = get_errors(
                gt_transform)
            # Store ground truth position and orientation (Yaw)
            gt_path.append((elapsed_time, gt_translation_error_x *
                           1e3, gt_translation_error_y * 1e3, gt_yaw))

            try:
                # Get estimated transform (for error calculation)
                t = handler.get_transform(gt_frame, est_frame)
            except Exception as e:
                rospy.logwarn(f"Failed to get estimated transform: {e}")
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

# After 60 seconds, save the error values and ground truth path to CSV files
rospy.loginfo("Saving error values and GT path to CSV files...")

cleaned_error_values = [row for row in error_values if row[1] is not None]
cleaned_gt_path = [row for row in gt_path if row[1] is not None]

# Write error values to a CSV file
csv_file = 'error_data.csv'
with open(csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time (seconds)", "Error X (mm)",
                    "Error Y (mm)", "Yaw (rad)"])
    writer.writerows(cleaned_error_values)

# Write GT path to a separate CSV file
gt_csv_file = 'gt_path.csv'
with open(gt_csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time (seconds)", "GT X (mm)",
                    "GT Y (mm)", "GT Yaw (rad)"])
    writer.writerows(cleaned_gt_path)

rospy.loginfo(
    f"Data saved to {csv_file} and ground truth path saved to {gt_csv_file}")
