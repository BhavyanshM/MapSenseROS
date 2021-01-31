
# Refer to the NVIDIA/CUDA base image here: (https://hub.docker.com/r/nvidia/cuda/)

FROM nvidia/cuda:11.2.0-base

ARG DEBIAN_FRONTEND=noninteractive

# Install commonly required libraries and packages
RUN apt-get update -y
RUN apt-get install -y nano git wget curl unzip apt-transport-https ca-certificates curl software-properties-common python3-opencv


# Setup ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update
RUN apt install -y ros-noetic-desktop-full
RUN apt-get install -y ros-noetic-ddynamic-reconfigure
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"


# Install OpenGL libraries on nvidia-docker2 
# Reference: (https://medium.com/@benjamin.botto/opengl-and-cuda-applications-in-docker-af0eece000f1)
RUN apt-get install -y opencl-headers ocl-icd-opencl-dev libgflags-dev libglvnd-dev libgl1-mesa-dev libegl1-mesa-dev libgles2-mesa-dev libxext6 libx11-6 freeglut3-dev mesa-utils


# Configure OpenCL (https://stackoverflow.com/questions/54243119/how-to-run-opencl-opengl-inside-a-docker-container)
RUN apt-get install -y clinfo
RUN mkdir -p /etc/OpenCL/vendors && echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

# Setup MapSenseROS Dependencies (Magnum, Corrade and MagnumIntegration for GUI and Rendering) 
# Reference: [https://doc.magnum.graphics/magnum/getting-started.html]
RUN apt-get install -y libopenal-dev libglfw3-dev libsdl2-dev libglm-dev
RUN mkdir Temp && cd Temp
RUN git clone git://github.com/mosra/corrade && cd corrade && ln -s package/debian . && dpkg-buildpackage && dpkg -i ../corrade*.deb && cd ..
RUN git clone git://github.com/mosra/magnum && cd magnum && ln -s package/debian . && dpkg-buildpackage &&  dpkg -i ../magnum*.deb && cd ..
RUN git clone git://github.com/mosra/magnum-integration && cd magnum-integration && cd src/MagnumExternal && \
	git clone https://github.com/ocornut/imgui.git && mv imgui ImGui && cd ../../ && \
	sed -i "s/IMGUI=OFF/IMGUI=ON/g" package/debian/rules && \
	sed -i "s/imgui_demo/imgui_demo imgui_tables/g" modules/FindImGui.cmake && \
	ln -s package/debian . && dpkg-buildpackage && dpkg -i ../magnum-integration*.deb


# Setup Catkin Workspace and MapSenseROS C++ node
RUN mkdir -p Workspace/catkin_ws/src


#Configure NVIDIA runtime for Docker
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute


COPY ./setup.sh /setup.sh


