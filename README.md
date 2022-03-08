# Armadillo2-Docker
Dockerfile, extentions and instructions on how to deploy an image to run Armadillo2 simulation via docker.

## Install docker
Follow the instructions [here](https://docs.docker.com/engine/install/ubuntu/) (those instruction are for Ubnutu OS).
It is recommended to manage Docker as a non-root user you can follow [this](https://docs.docker.com/engine/install/linux-postinstall/).
 
### Prerequisties
Install nvidia driver and CUDA. If you would like to work with deep learning as well you shuold install cudnn as well (e.g. using YOLO, real-time object detection).
You may uses [these](https://github.com/TalFeiner/bash_tools) scripts to install nvidia driver, CUDA and cudnn.

## Donwload Armadillo2 packeges
The require packeges to deploy the simulation are: 
1. [armadillo2](https://github.com/bguplp/armadillo).
2. [robotican demos upgrade](https://github.com/bguplp/robotican_demos_upgrade).
3. [gazebo worlds](https://github.com/bguplp/gazebo_worlds).
4. [gazebo_models](https://github.com/robotican/gazebo_models.git).

**Note** that repositories 1,2 are private, in order to use `git clone` command you need to connect through an ssh connection. 

You can clone the repositories with
```bash
git clone <repository>
```
Gather those repositories in one new folder (let's called it `docker_src`). For example,
```bash
cd ~/
mkdir -p armadillo2_ws/docker_src
cd armadillo2_ws/docker_src
# This clone is made through an ssh connection.
git clone git@github.com:bguplp/robotican_demos_upgrade.git
```

After you download all the packges, download the Dockerfile,
```bash
cd ~/armadillo2_ws/docker_src 
# If needed change the path to your's repositories directory.
wget https://github.com/bguplp/Armadillo2-Docker/raw/main/Dockerfile
```
Now there is a change we should do inside armadillo PID gains,
```bash
wget https://github.com/bguplp/Armadillo2-Docker/raw/main/armadillo2_control_gazebo.yaml && mv armadillo2_control_gazebo.yaml armadillo/armadillo2_control/config/
```

## Create a base image

1. If you didn't download the `Dockerfile` in the previous section download it to `docker_src` folder. Open a new terminal, navigate to `docker_src`
```bash
cd <path>/<to>/docker_src
```
2. Ensure that the host CUDA version and the docker base image are correlated. Verifty the HOST version with, 
```bash
cat /usr/local/cuda/version.txt
```
For example,

![alt text](https://github.com/bguplp/Armadillo2-Docker/blob/main/images/cuda_version.png)


Then build the image:
```bash
docker build -t armadillo2 . 
```
`armadillo2` is the image's name, the `-t` flag tags our image. And the `.` at the end, tells that Docker should look for the `Dockerfile` in the current directory.

## Setup and compile armadillo

## Run container with armadillo

### Update the current container
If you done some changes inside the container, and you would like to updates the image you can run
```bash
docker commit <container_name> <image_name>
```
The `container_name` can be found from VS-code (with docker extension) or
```bash
docker ps
```
Alternative 
```bash
docker container ls
```
The `image_name` is your own preference if you like to rewrite the image use the same name, you can also add tag by `<image_name>:<tag>`.
## Host container communication

## Trobleshooting
There is a lot, for now just do google search.
