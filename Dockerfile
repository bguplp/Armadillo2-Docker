# This docker create by Orel Hamamy, under BGU-CS department. 
# This image mount image with armadillo2 simulation, 
# For installation instruction please refer to: https://github.com/bguplp/Armadillo2-Docker/


# Installation of nvidia-libglvnd -------- 
FROM nvidia/opengl:1.2-glvnd-devel-ubuntu16.04

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,display

RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        ca-certificates \
        build-essential \
        g++ \
        libxinerama-dev \
        libxext-dev \
        libxrandr-dev \
        libxi-dev \
        libxcursor-dev \
        libxxf86vm-dev \
        libvulkan-dev && \
    rm -rf /var/lib/apt/lists/*

# ROS kinect installation ------- 
# generated from docker_images/create_ros_image.Dockerfile.em
FROM osrf/ros:kinetic-desktop-full

# install ros packages and ca-certificates for enable using wget
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kinetic-desktop=1.3.2-0* \
		ca-certificates \
		python3-pip \
		python-pip \
		wget\
    && rm -rf /var/lib/apt/lists/* 

# Install Gazebo7
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \
               `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
               && wget --no-check-certificate https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - 
               

RUN apt-get update && apt-get install -y gazebo7 libgazebo7-dev \
    ros-kinetic-gazebo-ros-pkgs \
    ros-kinetic-gazebo-ros-control\
    && rm -rf /var/lib/apt/lists/*

# Set setting for WS

WORKDIR /home/ubuntu
# Creating WS
RUN mkdir -p catkin_ws/src

# Copy reposetories and models
COPY . catkin_ws/src

#ADD .gazebo /.gazebo
RUN mkdir -p .gazebo/models 
RUN cp -a  catkin_ws/src/gazebo_worlds/Building_37/models .gazebo/models

RUN echo "export HOME=/home/ubuntu \nsource /opt/ros/kinetic/setup.bash\nsource /home/ubuntu/catkin_ws/devel/setup.bash" >> /root/.bashrc


# RUN source /root/.bashrc


# Install armadillo2, compatible with python3 alongside python2
# RUN ./catkin_ws/src/armadillo/armadillo2/docker_setup_py2_alongside_py3.sh
#WORKDIR /home/ubuntu/catkin_ws
#RUN catkin_make
#WORKDIR /home/ubuntu/catkin_ws/src/armadillo/armadillo2
#RUN rosdep update 
#RUN ["/bin/bash", "-c", "./docker_setup_py2_alongside_py3.sh"]
WORKDIR /home/ubuntu/catkin_ws

# RUN catkin_make




