#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, sin, cos
from tf.transformations import quaternion_from_euler


def send_waypoint(client, x, y, theta):
    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    quat = quaternion_from_euler(0, 0, radians(theta))
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    rospy.loginfo(f"Sending waypoint: x={x}, y={y}, theta={theta}")
    client.send_goal(goal)

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

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        client.wait_for_server()
        rospy.loginfo("Connected to move_base action server!")

        waypoints = [
            (3.5, -4.5, 0.0),   # Waypoint 1
            (4.5, -1.9, 90.0),  # Waypoint 2
            (6.7, -2.75, 180.0),  # Waypoint 3
            (5.82, -6.16, 270.0)  # Waypoint 4
        ]

        for waypoint in waypoints:
            x, y, theta = waypoint
            success = send_waypoint(client, x, y, theta)

            if not success:
                rospy.logwarn("Aborting waypoint navigation due to failure.")
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
