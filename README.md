# Armadillo2-Docker
Dockerfile, extentions and instructions on how to deploy an image to run Armadillo2 simulation via docker.

## Install docker
Follow the instructions [here](https://docs.docker.com/engine/install/ubuntu/) (those instruction are for Ubnutu OS).
It is recommended to manage Docker as a non-root user you can follow [this](https://docs.docker.com/engine/install/linux-postinstall/) procedure.
 

### Prerequisties
Install nvidia driver and CUDA. If you would like to work with deep learning as well you shuold install cudnn as well (e.g. using YOLO, real-time object detection).
You may uses [these](https://github.com/TalFeiner/bash_tools) scripts to install nvidia driver, CUDA and cudnn.

To run GPU accelerated containers we need to install the NVIDIA Container ToolKit installation guide can be found [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installing-on-ubuntu-and-debian).

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

We would like to use 11.0.* now let's updates the FROM image by changing line 9 from `FROM nvidia/cudagl:11.2.2-base-ubuntu16.04` to `FROM nvidia/cudagl:11.0.3-base-ubuntu16.04`

You may find your specific version [here](https://hub.docker.com/r/nvidia/cudagl/tags).
3. build the image:
```bash
docker build -t armadillo2 . 
```
`armadillo2` is the image's name, the `-t` flag tags our image. And the `.` at the end, tells that Docker should look for the `Dockerfile` in the current directory. This would take a few minutes. We can verifty is succesed with vscode (through docker extension) or with `docker images`

![alt text](https://github.com/bguplp/Armadillo2-Docker/blob/main/images/images_list.png)

Now we ready for the next step, installation and compilation of armadillo2 simulation.

## Setup and compile armadillo

Download the script for running a container, this script run a container with the essential arguments.
```bash 
wget https://github.com/bguplp/Armadillo2-Docker/raw/main/run_container.bash
chmod +x run_container.bash 
```
If you set the image different name then `armadillo2` you can pass an argument for this script with the image name
```bash 
./run_container.bash <image_tag>
```
The default `image_tag` is `armadillo2`.

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
There is a few, I didn't managed to documents a lot so you may try google search or contact me via email [orelhamamy@gamil.com](mailto:orelhamamy@gamil.com?subject=[github]%20Armadillo2%20Docker)
