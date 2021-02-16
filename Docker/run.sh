docker run -it \
	--name mapsense \
	--net=host \
	--env="DISPLAY" \
	--volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume "$(pwd)/SharedVolume:/home/robotlab/SharedVolume:rw" \
	--volume "$(pwd)/MapSenseROS:/home/robotlab/dev/mapsense_ws/src/MapSenseROS:rw" \
	--volume "$(pwd)/JetBrains:/home/robotlab/.config/JetBrains:rw" \
	--privileged \
	--runtime=nvidia \
	--gpus all \
	--device /dev/dri:/dev/dri \
	bmishra/mapsense-nvidia-ros:latest bash
