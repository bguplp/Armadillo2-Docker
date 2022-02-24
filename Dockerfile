# This docker create by Orel Hamamy, under BGU-CS department. 
# This image mount image with armadillo2 simulation, 
# For installation instruction please refer to: https://github.com/bguplp/Armadillo2-Docker/


# Installation of nvidia-libglvnd -------- 
#FROM nvidia/cudagl:10.2-base-ubuntu16.04
FROM nvidia/cudagl:10.1-base-ubuntu16.04

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
# Register the ROS package sources.

ENV UBUNTU_RELEASE=xenial
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $UBUNTU_RELEASE main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install ros packages and ca-certificates for enable using wget
RUN apt-get update && apt-get install -y \
        ros-kinetic-desktop-full \ 
    && rm -rf /var/lib/apt/lists/* 
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        python3-pip \
        python-pip \
        wget\
    && rm -rf /var/lib/apt/lists/* 
RUN rosdep init

# Install Gazebo7
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \
               `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
               && wget --no-check-certificate https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - 
RUN apt-get update && apt-get install -y gazebo7 \
#    libgazebo7-dev \
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
 && mkdir -p /catkin_ws \
 && ln -s /catkin_ws /home/catkin_ws \
 && chown -R ros:ros /home/ros /catkin_ws

# Source the ROS configuration.
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/ros/.bashrc

# If the script is started from a Catkin workspace,
# source its configuration as well.
RUN echo "test -f devel/setup.bash && echo \"Found Catkin workspace.\" && source devel/setup.bash" >> /home/ros/.bashrc

USER ros
RUN rosdep update --include-eol-distros

WORKDIR /home/ros/catkin_ws

# Set setting for WS
# Creating WS
RUN mkdir -p src

# Copy reposetories and models
COPY . src

#ADD .gazebo /.gazebo
RUN mkdir -p /home/ros/.gazebo
RUN cp -a  src/gazebo_worlds/Building_37/models/ /home/ros/.gazebo/

RUN echo "export HOME=/home/ros \nsource /opt/ros/kinetic/setup.bash\nsource /home/ros/catkin_ws/devel/setup.bash" >> /home/ros/.bashrc


# RUN source /root/.bashrc


# Install armadillo2, compatible with python3 alongside python2
# RUN ./catkin_ws/src/armadillo/armadillo2/docker_setup_py2_alongside_py3.sh
#WORKDIR /home/ubuntu/catkin_ws
#RUN catkin_make
#WORKDIR /home/ubuntu/catkin_ws/src/armadillo/armadillo2
#RUN rosdep update 
#RUN ["/bin/bash", "-c", "./docker_setup_py2_alongside_py3.sh"]
# RUN rosdep update

# RUN catkin_make




