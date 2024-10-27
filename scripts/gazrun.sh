#!/bin/bash

# Source ROS and workspace
source ~/.bashrc

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=waffle

# Launch the simulation and custom node
roslaunch turtlebot3 gazrun.launch
