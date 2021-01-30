sudo xhost +local:docker

sudo docker run -it \
	--net=host \
	--env="DISPLAY" \
	--volume "/tmp/.X11-unix:/tmp/.X11-unix" \
	--volume "/home/quantum/Workspace:/Source" \
	ihmc