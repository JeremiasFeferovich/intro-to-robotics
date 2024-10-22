#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv
import time

amcl_x_positions = []
amcl_y_positions = []
timestamps = []

def amcl_pose_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    
    amcl_x_positions.append(x)
    amcl_y_positions.append(y)
    timestamps.append(time.time() - start_time)

def amcl_listener():
    global start_time
    
    rospy.init_node('amcl_listener', anonymous=True)
    
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    
    start_time = time.time()
    
    duration = 60
    rospy.loginfo("Collecting (x, y) position data from AMCL (estimated) for 1 minute...")
    
    while time.time() - start_time < duration:
        rospy.sleep(0.1)  
    
    rospy.loginfo("Data collection complete. Saving to CSV...")
    save_to_csv()

def save_to_csv():
    filename = 'amcl_positions.csv'
    
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        writer.writerow(['Timestamp', 'X Position', 'Y Position'])
        
        for i in range(len(amcl_x_positions)):
            writer.writerow([timestamps[i], amcl_x_positions[i], amcl_y_positions[i]])
    
    rospy.loginfo(f"AMCL position data saved to {filename}")

if __name__ == '__main__':
    try:
        amcl_listener()
    except rospy.ROSInterruptException:
        pass

