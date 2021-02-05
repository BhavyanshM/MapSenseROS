#! /bin/bash


if [[ $1 = "clone" ]]
then
	echo "Cloning MapSenseROS:"
	cd /Workspace/catkin_ws/src
	git clone https://github.com/BhavyanshM/MapSenseROS.git

elif [[ $1 = "pull" ]]
then
	echo "Pulling Latest MapSenseROS:"
	cd MapSenseROS
	git pull origin master
	cd ..

elif [[ $1 = "dev" ]]; 
then
	cd /Workspace/catkin_ws
	catkin_make
	exit 0

else
	echo "Usage: compile.sh [clone/pull/dev]"
	exit 0
fi

cd /Workspace/catkin_ws
catkin_make

