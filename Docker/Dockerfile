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

# Install graphics and UI tools magnum and ImGui
USER robotlab
RUN mkdir dev
WORKDIR /home/robotlab/dev
USER root

# Install latest GTSAM from source
RUN cd /tmp && \
    wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB && \
    apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB && \
    rm GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB && \
    cd - && \
    wget https://apt.repos.intel.com/setup/intelproducts.list -O /etc/apt/sources.list.d/intelproducts.list

RUN add-apt-repository ppa:borglab/gtsam-develop \
 && apt-get --quiet 2 --yes update \
 && apt-get --quiet 2 --yes install \
    libvtk7-dev \
    libboost-all-dev \
    libtbb-dev \
    intel-mkl \
    libgtsam-dev \
    libgtsam-unstable-dev \
    dpkg-dev \
    debhelper \
    libxrandr-dev \
    libxinerama-dev \
    > /dev/null

RUN apt-get --quiet 2 --yes install \
    libxcursor-dev \
    libxi-dev \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*




# Setup MapSenseROS Dependencies (Magnum, Corrade and MagnumIntegration for GUI and Rendering)
# Reference: [https://doc.magnum.graphics/magnum/getting-started.html]
# RUN apt-get --quiet 2 --yes update \
#  && apt-get --quiet 2 --yes install \
#     libopenal-dev \
#     libglfw3-dev \
#     libsdl2-dev \
#     libglm-dev \
#     dpkg-dev \
#     debhelper \
#     libbullet-dev \
#     > /dev/null \
#  && rm -rf /var/lib/apt/lists/*


RUN mkdir packages \
    && cd packages \
    && git clone --recurse-submodules -j8 git://github.com/BhavyanshM/ClayEngine \
    && cd ClayEngine \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j32 \
    && make install \
    && cd ../../.. \
    && rm -rf packages

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
