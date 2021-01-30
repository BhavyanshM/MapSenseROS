sudo xhost +local:docker

sudo docker run -it \
	--net=host \
	--env="DISPLAY" \
	--volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume "/home/quantum/Workspace:/Source" \
	--privileged \
	--runtime=nvidia \
	--rm --gpus all \
	--device /dev/dri:/dev/dri \
	bmishra/mapsense-nvidia-ros bash setup.sh