#!/bin/bash
# Uncomment for debugging this script
set -o xtrace

# Make sure it works one way or the other to reduce possible errors
if (( EUID == 0 )); then
    echo "Run without sudo." 1>&2
    exit 1
fi

sudo -u $(whoami) xhost +local:docker

OS_NAME=$(awk -F= '/^NAME/{print $2}' /etc/os-release)
OS_NAME="${OS_NAME%\"}"
OS_NAME="${OS_NAME#\"}"

echo "OS: $OS_NAME"

if [[ "$OS_NAME" == *"Arch"* ]]; then
  sudo -u root docker run -it \
  	--name mapsense \
  	--net=host \
  	--env="DISPLAY" \
    --rm \
  	--volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  	--volume "$(pwd)/../.."/SharedVolume:/home/robotlab/SharedVolume:rw \
  	--volume "$(pwd)/..":/home/robotlab/dev/mapsense_ws/src/MapSenseROS:rw \
  	--volume "$HOME"/.config/JetBrains:/home/robotlab/.config/JetBrains:rw \
  	--privileged \
  	--gpus all \
  	--device /dev/dri:/dev/dri \
  	ihmcrobotics/mapsense-nvidia-ros:0.8 /home/robotlab/dev/mapsense_ws/src/MapSenseROS/scripts/startMapSenseCLion.sh
else
  sudo -u root docker run -it \
    --name mapsense \
    --net=host \
    --env="DISPLAY" \
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume "$(pwd)/../.."/SharedVolume:/home/robotlab/SharedVolume:rw \
    --volume "$(pwd)/..":/home/robotlab/dev/mapsense_ws/src/MapSenseROS:rw \
    --volume "$HOME"/.config/JetBrains:/home/robotlab/.config/JetBrains:rw \
    --privileged \
    --runtime=nvidia \
    --gpus all \
    --device /dev/dri:/dev/dri \
    ihmcrobotics/mapsense-nvidia-ros:0.8 /home/robotlab/dev/mapsense_ws/src/MapSenseROS/scripts/startMapSenseCLion.sh
fi
