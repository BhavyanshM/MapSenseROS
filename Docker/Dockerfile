
# Refer to the NVIDIA/CUDA base image here: (https://hub.docker.com/r/nvidia/cuda/)

FROM nvidia/cuda:11.2.0-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y

RUN apt-get install -y nano git wget curl unzip apt-transport-https ca-certificates curl software-properties-common python3-opencv


# Setup ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update
RUN apt install ros-noetic-desktop-full
RUN apt-get install -y ros-noetic-ddynamic-reconfigure
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

# Setup MapSenseROS Source Code
RUN mkdir Workspace
RUN cd /Workspace/ && git clone https://github.com/BhavyanshM/MapSenseROS.git


COPY ./setup.sh /Workspace/setup.sh
COPY ./magnum-setup.sh /Workspace/magnum-setup.sh


