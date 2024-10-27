#!/bin/bash

# Source ROS and workspace
source ~/.bashrc

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=waffle

# Value parameter 
VALUE=$1
echo "Value: $VALUE"

# Launch the simulation and custom node
roslaunch turtlebot3 move.launch distance:=$VALUE