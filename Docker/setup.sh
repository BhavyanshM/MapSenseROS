#! /bin/bash


alias mapsense="rosrun map_sense planar_region_publisher"
alias clion-mapsense="/opt/clion/bin/clion.sh /Workspace/catkin_ws/src </dev/null &>/dev/null &"

source /opt/ros/noetic/setup.bash 
roscore </dev/null &>/dev/null &

rm -rf /corrade* /magnum*