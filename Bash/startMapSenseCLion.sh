#!/bin/bash
set -o xtrace

cd /home/robotlab/dev/mapsense_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
clion src
