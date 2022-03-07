# Armadillo2-Docker
Dockerfile with instructions on how to deploy an image to run Armadillo2 simulation via docker

## Install docker

## Donwload Armadillo2 packeges
The require packeges to deploy the simulation are: 
1. [armadillo2](https://github.com/bguplp/armadillo).
2. [robotican demos upgrade](https://github.com/bguplp/robotican_demos_upgrade).
3. [gazebo worlds](https://github.com/bguplp/gazebo_worlds).
4. [gazebo_models](https://github.com/robotican/gazebo_models.git).
You can clone the repositories with
```bash
git clone <repository>
```
Gather those repositories in one new folder (let's called it `docker_src`).

## Create a base image

Download the `Dockerfile` to `docker_src` folder. Now open a new terminal, cd to `docker_src`
```bash
cd <path>/<to>/docker_src
```
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
