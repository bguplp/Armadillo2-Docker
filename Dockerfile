# This docker create by Orel Hamamy, under BGU-CS department. 
# This image mount image with armadillo2 simulation, 
# For installation instruction please refer to: https://github.com/bguplp/Armadillo2-Docker/


# Installation of nvidia-cudagl -------- 
#FROM nvidia/cudagl:10.2-base-ubuntu16.04
#FROM nvidia/cudagl:10.1-base-ubuntu16.04
FROM nvidia/cudagl:11.0.3-base-ubuntu16.04


# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,display


RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    mesa-utils \
    vim nano\
    build-essential gdb \
    cmake cmake-curses-gui \
    git \
    ssh \
 && rm -rf /var/lib/apt/lists/*

# -------- ROS kinetic installation ------- 
# Register the ROS package sources.

ENV UBUNTU_RELEASE=xenial
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $UBUNTU_RELEASE main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ros
RUN apt-get update && apt-get install -y \
        ros-kinetic-desktop-full \ 
    && rm -rf /var/lib/apt/lists/* 

RUN rosdep init

# Install Gazebo7
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \
               `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
               && wget --no-check-certificate https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - 
RUN apt-get update && apt-get install -y gazebo7 \
    ros-kinetic-gazebo-ros-pkgs \
    ros-kinetic-gazebo-ros-control\
    && rm -rf /var/lib/apt/lists/*

# Some QT-Apps/Gazebo don't show controls without this
ENV QT_X11_NO_MITSHM 1

# Create users and groups.
ARG ROS_USER_ID=1000
ARG ROS_GROUP_ID=1000

RUN addgroup --gid $ROS_GROUP_ID ros \
 && useradd --gid $ROS_GROUP_ID --uid $ROS_USER_ID -ms /bin/bash -p "$(openssl passwd -1 ros)" -G root,sudo ros \
 && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
 && chown -R ros:ros /home/

# Source the ROS configuration.
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/ros/.bashrc

# If the script is started from a Catkin workspace,
# source its configuration as well.
RUN echo "test -f ~/catkin_ws/devel/setup.bash && { source ~/catkin_ws/devel/setup.bash ; echo "catkin_ws was found!"; }" >> /home/ros/.bashrc

# Creating WS
WORKDIR /home/ros/catkin_ws
RUN mkdir -p src

# ROS dep update
USER ros
RUN rosdep update --include-eol-distros

# Install pip & pip3
RUN curl -fsSL https://bootstrap.pypa.io/pip/3.5/get-pip.py | sudo python3.5 \
    && curl -fsSL https://bootstrap.pypa.io/pip/2.7/get-pip.py | sudo python
    
# Copy reposetories and models
COPY . src

#copy files  .gazebo /.gazebo
RUN mkdir -p /home/ros/.gazebo
RUN cp -a src/gazebo_worlds/Building_37/models/ /home/ros/.gazebo/
RUN cp -a src/gazebo_models/* /home/ros/.gazebo/models/

# Change file owner for permissions privilege
RUN sudo chown -R ros:ros /home/

