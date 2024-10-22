#!/bin/bash

# Run roscore in a new terminal using zsh
gnome-terminal -- zsh -c "roscore; exec zsh"

# Wait for roscore to start
sleep 5

# Run TurtleBot3 setup and launch in another terminal using zsh
gnome-terminal -- zsh -c "cd ~/intro-to-robotics && catkin_make && source devel/setup.zsh && roslaunch turtlebot3_datasets turtlebot3_playbag.launch; exec zsh"

# Wait for the launch to stabilize
sleep 5

# Run the Python script in another terminal using zsh
gnome-terminal -- zsh -c "python3 ~/intro-to-robotics/src/turtlebot3_datasets/scripts/calculate_error_and_extract.py; exec zsh"
