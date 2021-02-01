sudo xhost +local:docker


mkdir Shared_Volume #To store ROSBags for testing MapSense

sudo docker run -it \
	--net=host \
	--env="DISPLAY" \
	--volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume ${pwd}/Shared_Volume:/Shared_Volume \
	--privileged \
	--runtime=nvidia \
	--rm --gpus all \
	--device /dev/dri:/dev/dri \
	bmishra/mapsense-nvidia-ros bash setup.sh