#! /bin/bash

source /opt/ros/noetic/setup.bash 
rm -rf corrade* magnum*
cd /Workspace/catkin_ws/src && git clone https://github.com/BhavyanshM/MapSenseROS.git
cd /Workspace/catkin_ws/
catkin_make
source devel/setup.bash 
rosrun map_sense planar_region_publisher