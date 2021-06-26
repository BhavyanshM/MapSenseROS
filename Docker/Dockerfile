# Current version: 0.8
FROM ihmcrobotics/nvidia-ros:0.3

USER root

ARG DEBIAN_FRONTEND=noninteractive

# TODO: Use (https://code.visualstudio.com/docs/remote/containers) to configure VSCode for containers
# CLion (https://www.jetbrains.com/clion/download/#section=linux)
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
RUN apt-get --quiet 2 --yes update \
 && apt-get --quiet 2 --yes install \
    libopenal-dev \
    libglfw3-dev \
    libsdl2-dev \
    libglm-dev \
    dpkg-dev \
    debhelper \
    libbullet-dev \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

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
 && cd ImGui && git checkout docking && cd ../../../ \
 && sed -i "s/IMGUI=OFF/IMGUI=ON/g" package/debian/rules \
 && sed -i "s/imgui_demo/imgui_demo imgui_tables/g" modules/FindImGui.cmake \
 && ln -s package/debian . \
 && dpkg-buildpackage > /dev/null \
 && dpkg -i ../magnum-integration*.deb \
 && cd ../.. \
 && rm -rf magnum-integration

# Setup Catkin Workspace and MapSenseROS C++ node
RUN apt-get --quiet 2 --yes update \
 && apt-get --quiet 2 --yes install \
    libvtk7-dev \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# Install latest GTSAM from source
RUN cd /tmp && \
    wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB && \
    apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB && \
    rm GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB && \
    cd - && \
    wget https://apt.repos.intel.com/setup/intelproducts.list -O /etc/apt/sources.list.d/intelproducts.list

RUN apt-get --quiet 2 --yes update \
 && apt-get --quiet 2 --yes install \
    libboost-all-dev \
    libtbb-dev \
    intel-mkl \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository ppa:borglab/gtsam-develop \
 && apt-get --quiet 2 --yes update \
 && apt-get --quiet 2 --yes install \
    libgtsam-dev \
    libgtsam-unstable-dev \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

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
