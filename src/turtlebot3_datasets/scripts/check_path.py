#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv
import time

# Lists to store x, y positions, and timestamps from AMCL (estimated)
amcl_x_positions = []
amcl_y_positions = []
timestamps = []

def amcl_pose_callback(msg):
    # Extract the x and y positions from AMCL (estimated position)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    
    # Append the AMCL x, y positions and current time to their respective lists
    amcl_x_positions.append(x)
    amcl_y_positions.append(y)
    timestamps.append(time.time() - start_time)

def amcl_listener():
    global start_time
    
    # Initialize the ROS node
    rospy.init_node('amcl_listener', anonymous=True)
    
    # Subscribe to the AMCL pose topic (estimated position)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    
    # Start time (reference for timestamps)
    start_time = time.time()
    
    # Run for 1 minute (60 seconds)
    duration = 60
    rospy.loginfo("Collecting (x, y) position data from AMCL (estimated) for 1 minute...")
    
    while time.time() - start_time < duration:
        rospy.sleep(0.1)  # Sleep for a short time to allow callback processing
    
    # After 1 minute, save the data to a CSV file
    rospy.loginfo("Data collection complete. Saving to CSV...")
    save_to_csv()

def save_to_csv():
    # Define the CSV file name
    filename = 'amcl_positions.csv'
    
    # Open the CSV file in write mode
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # Write the header
        writer.writerow(['Timestamp', 'X Position', 'Y Position'])
        
        # Write the AMCL data (timestamps, x, y positions)
        for i in range(len(amcl_x_positions)):
            writer.writerow([timestamps[i], amcl_x_positions[i], amcl_y_positions[i]])
    
    rospy.loginfo(f"AMCL position data saved to {filename}")

if __name__ == '__main__':
    try:
        amcl_listener()
    except rospy.ROSInterruptException:
        pass

