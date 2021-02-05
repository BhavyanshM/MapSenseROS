xhost +local:docker


docker run -it \
	--name mapsense \
	--net=host \
	--env="DISPLAY" \
	--volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume "${pwd}/Shared_Volume:/Shared_Volume:rw" \
	--privileged \
	--runtime=nvidia \
	--gpus all \
	--device /dev/dri:/dev/dri \
	bmishra/mapsense-nvidia-ros:latest bash