# Uncomment for debugging this script
# set -o xtrace

OS_NAME=$(awk -F= '/^NAME/{print $2}' /etc/os-release)
OS_NAME="${OS_NAME%\"}"
OS_NAME="${OS_NAME#\"}"

echo "OS: $OS_NAME"

if [[ "$OS_NAME" == *"Arch"* ]]; then
  docker run -it \
  	--name mapsense \
  	--net=host \
  	--env="DISPLAY" \
  	--volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  	--volume "$(pwd)/../.."/SharedVolume:/home/robotlab/SharedVolume:rw \
  	--volume "$(pwd)/../..":/home/robotlab/dev/mapsense_ws/src:rw \
  	--volume "$HOME"/.config/JetBrains:/home/robotlab/.config/JetBrains:rw \
  	--privileged \
  	--gpus all \
  	--device /dev/dri:/dev/dri \
  	ihmcrobotics/mapsense-nvidia-ros:0.6 bash
else
  docker run -it \
    --name mapsense \
    --net=host \
    --env="DISPLAY" \
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume "$(pwd)/../.."/SharedVolume:/home/robotlab/SharedVolume:rw \
    --volume "$(pwd)/../..":/home/robotlab/dev/mapsense_ws/src:rw \
    --volume "$HOME"/.config/JetBrains:/home/robotlab/.config/JetBrains:rw \
    --privileged \
    --runtime=nvidia \
    --gpus all \
    --device /dev/dri:/dev/dri \
    ihmcrobotics/mapsense-nvidia-ros:0.6 bash
fi
