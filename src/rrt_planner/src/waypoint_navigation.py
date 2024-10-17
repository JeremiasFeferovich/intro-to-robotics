#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, sin, cos
from tf.transformations import quaternion_from_euler


def send_waypoint(client, x, y, theta):
    # Create a goal to send to move_base
    goal = MoveBaseGoal()

    # Set the frame of reference to map
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the x, y, and orientation (yaw converted to quaternion) of the waypoint
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    quat = quaternion_from_euler(0, 0, radians(theta))
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    rospy.loginfo(f"Sending waypoint: x={x}, y={y}, theta={theta}")
    client.send_goal(goal)

    # Wait for the robot to reach the waypoint or fail
    client.wait_for_result()

    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Reached waypoint!")
        return True
    else:
        rospy.logwarn("Failed to reach waypoint.")
        return False


if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_navigation')

        # Create a SimpleActionClient for move_base
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        client.wait_for_server()
        rospy.loginfo("Connected to move_base action server!")

        # Define your waypoints here (x, y, yaw in degrees)
        waypoints = [
            (0.7, 1.2, 0.0),   # Waypoint 1
            (2.0, 1.0, 90.0),  # Waypoint 2
            (2.0, 2.0, 180.0),  # Waypoint 3
            (1.0, 2.0, 270.0)  # Waypoint 4
        ]

        # Loop through each waypoint and send the next after completing the previous one
        for waypoint in waypoints:
            x, y, theta = waypoint
            success = send_waypoint(client, x, y, theta)

            if not success:
                rospy.logwarn("Aborting waypoint navigation due to failure.")
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
