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
4. [gazebo models](https://github.com/robotican/gazebo_models.git).
5. [armadillo navigation upgrade](https://github.com/bguplp/armadillo_navigation_upgrade)
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
If you set the image different name then `armadillo2` you can pass an argument for this script with the image name,
```bash 
sudo ./run_container.bash <image_tag>
```
The default `image_tag` is `armadillo2`.

Navigate to armadillo2 folder,
```bash 
cd src/armadillo/armadillo2
```
The absolute path is `/home/ros/catkin_ws/src/armadillo/armadillo2`. Run the setup file,
```bash 
./docker_setup_py2_alongside_py3.sh
```
This would take some time...

After the setup is done, we commiting to the image **This execute within a host terminal**,
```bash
docker commit [OPTIONS] CONTAINER [REPOSITORY[:TAG]]
```
To find out your CONTAINER ID, run `docker ps`. Either use autocomplete (Tab key) to use the container name. The repository may be again armadillo2. Example for a commit command,
```bash
docker commit xenodochial_varahamihira armadillo2
```

Now let's setup our environment by source `.bashrc`,
```bash
source ~/.bashrc
```


## Run container with armadillo

After you build the image, setup armadillo2 within the container and commit, you able to run a container. Now we will demonstrate how to runs a container, inside the container we will implement simulation, launch a file and excute pick operation. 

1. Run a container,
```bash 
sudo ./run_container.bash <image_tag>
```
2. Start armadillo2 simulation,
```
roslaunch armadillo2 armadillo2.launch gazebo:=true kinect:=true world_name:="`rospack find armadillo2_gazebo`/worlds/building_37_sim_1.world" moveit:=true x:=-17.08 y:=18.55 z:=2.598
```
3. Open new terminal within the container with,
```bash
docker exec -it <container_name> bash
```
This step will redo every time when we need a new terminal.

4. (recommend) Inside the container running a `rqt`, to navigates and monitoring the robot.

5. Launch the script for pickup,
```bash
roslaunch robotican_demos_upgrade sim_launch.launch 
```
Notice, I changed this script to work with color detection and not YOLO.

6. Navigate the robot in front of the can using the `rqt`.

7. Call the pick service.
```bash 
rosservice call /pick_unknown "robot: ''
obj: ''
discrete_location: ''" 
```
The final result,
![alt text](https://github.com/bguplp/Armadillo2-Docker/blob/main/images/pickup-based-color.png)

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

While I used ROS noetic within the host machine it work fine, I manage to run simulation in a container, YOLO-4 on the host machine and pick-up services and launch inside the container, so I can say it can comunicate and transfer data, without additional configuration.

## Trobleshooting
There is a few, I didn't managed to documents a lot so you may try google search or contact me via email [orelhamamy@gamil.com](mailto:orelhamamy@gamil.com?subject=[github]%20Armadillo2%20Docker).
### Low Disk Space on "Filesystem root"

The docker storges data in `/var/lib/docker/` directory, which may cause consuming of a lot of space. There are two possibale solutions,
 1. Delete previous data like old containers and etc. 
```bash
docker system prune 
```
**NOTICE** You may add -a flag, be aware this command can delete your images as well.

 2. Change to docker data directory from `/root` to `/home`. For more detail see this [link](https://www.ibm.com/docs/en/z-logdata-analytics/5.1.0?topic=compose-relocating-docker-root-directory).
	2.1 Stop the Docker service,
```bash
sudo systemctl stop docker
sudo systemctl stop docker.socket
sudo systemctl stop containerd
```
	2.2 Create new dir for the docker,
```bash
sudo mkdir -p /new_dir_structure
```
	2.3 Move Docker root to the new dir,
```bash
sudo mv /var/lib/docker /new_dir_structure
```
	2.4 Edit (with root privilege) the file `/etc/docker/daemon.json`. If you installed Nvidia-Docker 2.0 the file should be existed, edit it as follow,
```JSON
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "data-root": "/<path>/<to>/<your>/<directory>"
}
```
	2.5 Restart the Docker services, 
```bash
sudo systemctl start docker
```
