
# Refer to the NVIDIA/CUDA base image here: (https://hub.docker.com/r/nvidia/cuda/)

FROM nvidia/cuda:11.2.0-base

ARG DEBIAN_FRONTEND=noninteractive

# Install commonly required libraries and packages
RUN apt-get --quiet 2 --yes update  \
 && apt-get --quiet 2 --yes install \
    nano \
    git \
    wget \
    curl \
    unzip \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common \
    python3-opencv \
    > /dev/null

# Setup ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
 && apt-get update --quiet 2 --yes \
 && apt-get install --quiet 2 --yes \
    ros-noetic-desktop \
    ros-noetic-ddynamic-reconfigure \
    > /dev/null

# Install OpenGL libraries on nvidia-docker2 
# Reference: (https://medium.com/@benjamin.botto/opengl-and-cuda-applications-in-docker-af0eece000f1)
RUN apt-get install --quiet 2 --yes \
    opencl-headers \
    ocl-icd-opencl-dev \
    libgflags-dev \
    libglvnd-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    libxext6 \
    libx11-6 \
    freeglut3-dev \
    mesa-utils \
    > /dev/null

# Configure OpenCL (https://stackoverflow.com/questions/54243119/how-to-run-opencl-opengl-inside-a-docker-container)
RUN apt-get install --quiet 2 --yes clinfo > /dev/null \
 && mkdir -p /etc/OpenCL/vendors \
 && echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

#Configure NVIDIA runtime for Docker
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

RUN addgroup robotlab \
 && adduser --home /home/robotlab --gecos "Rosie Robot,1117,1234567,2345678" --ingroup robotlab --disabled-password robotlab \
 && chown -R robotlab /home/robotlab
USER robotlab
WORKDIR /home/robotlab

# TODO: Use (https://code.visualstudio.com/docs/remote/containers) to configure VSCode for containers
# CLion (https://www.jetbrains.com/clion/download/#section=linux)
USER root
RUN mkdir -p Downloads \
 && cd Downloads \
 && curl -sL https://download.jetbrains.com/cpp/CLion-2021.1.tar.gz -o clion.tar.gz \
 && tar -xzf clion.tar.gz \
 && mv clion-2021.1/ /opt/clion \
 && ln -s /opt/clion/bin/clion.sh /usr/local/bin/clion \
 && cd .. \
 && rm -rf Downloads

# Setup MapSenseROS Dependencies (Magnum, Corrade and MagnumIntegration for GUI and Rendering)
# Reference: [https://doc.magnum.graphics/magnum/getting-started.html]
RUN apt-get install --quiet 2 --yes \
    libopenal-dev \
    libglfw3-dev \
    libsdl2-dev \
    libglm-dev \
    dpkg-dev \
    debhelper \
    libbullet-dev \
    > /dev/null
# Install graphics and UI tools magnum and ImGui
USER robotlab
RUN mkdir dev
WORKDIR /home/robotlab/dev
USER root
RUN mkdir corrade \
 && cd corrade \
 && git clone git://github.com/mosra/corrade \
 && cd corrade \
 && ln -s package/debian . \
 && dpkg-buildpackage > /dev/null \
 && dpkg -i ../corrade*.deb \
 && cd ../.. \
 && rm -rf corrade
RUN mkdir magnum \
 && cd magnum \
 && git clone git://github.com/mosra/magnum \
 && cd magnum \
 && ln -s package/debian . \
 && dpkg-buildpackage > /dev/null \
 && dpkg -i ../magnum*.deb \
 && cd ../.. \
 && rm -rf magnum
RUN mkdir magnum-integration \
 && cd magnum-integration \
 && git clone git://github.com/mosra/magnum-integration \
 && cd magnum-integration \
 && cd src/MagnumExternal \
 && git clone https://github.com/ocornut/imgui.git \
 && mv imgui ImGui \
 && cd ../../ \
 && sed -i "s/IMGUI=OFF/IMGUI=ON/g" package/debian/rules \
 && sed -i "s/imgui_demo/imgui_demo imgui_tables/g" modules/FindImGui.cmake \
 && ln -s package/debian . \
 && dpkg-buildpackage > /dev/null \
 && dpkg -i ../magnum-integration*.deb \
 && cd ../.. \
 && rm -rf magnum-integration

# Setup Catkin Workspace and MapSenseROS C++ node
RUN apt-get install --quiet 2 --yes \
    libvtk7-dev \
    > /dev/null

# Install latest GTSAM from source
RUN cd /tmp && \
    wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB && \
    apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB && \
    rm GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB && \
    cd - && \
    wget https://apt.repos.intel.com/setup/intelproducts.list -O /etc/apt/sources.list.d/intelproducts.list

RUN apt-get update --quiet 2 --yes && \
    apt-get install --quiet 2 --yes \
    libboost-all-dev \
    libtbb-dev \
    intel-mkl

RUN add-apt-repository ppa:borglab/gtsam-develop && \
    apt-get install --quiet 2 --yes \
    libgtsam-dev \
    libgtsam-unstable-dev


USER robotlab
RUN mkdir -p /home/robotlab/dev/mapsense_ws/src
VOLUME /home/robotlab/dev/mapsense_ws/src

RUN mkdir -p /home/robotlab/.config/JetBrains
VOLUME /home/robotlab/.config/JetBrains

RUN mkdir -p /home/robotlab/SharedVolume
VOLUME /home/robotlab/SharedVolume

WORKDIR /home/robotlab
COPY ./compile.sh /home/robotlab/compile.sh

RUN echo 'alias mapsense="source /home/robotlab/dev/mapsense_ws/devel/setup.bash && \
	      rosrun map_sense planar_region_publisher"' >> /home/robotlab/.bashrc
RUN echo 'alias mapsense-clion="source /opt/ros/noetic/setup.bash && source /home/robotlab/dev/mapsense_ws/devel/setup.bash && \
	      /opt/clion/bin/clion.sh /home/robotlab/dev/mapsense_ws/src </dev/null &>/dev/null &"' >> /home/robotlab/.bashrc
RUN echo 'source /opt/ros/noetic/setup.bash' >> /home/robotlab/.bashrc
RUN echo '#roscore </dev/null &>/dev/null &' >> /home/robotlab/.bashrc
RUN echo 'export ROS_MASTER_URI=http://localhost:11311/' >> /home/robotlab/.bashrc
