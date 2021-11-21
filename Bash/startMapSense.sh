#!/bin/bash
set -o xtrace

cd /home/robotlab/dev/mapsense_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rosrun map_sense planar_region_publisher
