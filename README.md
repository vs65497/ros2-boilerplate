# Instructions for setting up Docker and ROS2 on Raspberry Pi
Boilerplate ROS2 for Raspberry Pi

Last Updated: April 2, 2023

# Setting up Raspberry Pi
Using Raspberry Pi Imager, select:

- Ubuntu Server 22.04.2 LTS (64-bit) ARM x64
- Set hostname: "____.local"
- Enable SSH, Use password authentication
- Set username and password
- Configure wireless LAN
- SSID and Password: [wifi credentials]
- Wireless LAN country: US
- Set locale settings: America, us
- Play sound when finished, Eject media when finished, Enable telemetry

In order to connect the RPi to the network it was necessary to connect an ethernet cable to the pi on first initialization.

If you go to the router webpage, you should be able to see the new pi come online under the wireless section. If all goes well then the hostname of the pi will also be updated.

This didn't work every time. I just reflashed the SD card and tried again with the pi connected directly to the router.

Take note of the IP address. I was typically not able to find the pi with a normal scan of the router network.

`nmap -sn 192.168.1.0/24`

To connect:

`ssh [username]@[ip address]`

Usually I make a shell script for this saved as [username].sh just so I don't forget the IP address.

# Setup Docker and Portainer.io
(From: https://www.youtube.com/watch?v=O7G3oatg5DA)

```
sudo apt update
sudo apt upgrade

curl -sSL https://get.docker.com | sh

sudo docker pull portainer/portainer-ce:linux-arm

sudo docker run -d -p 9000:9000 --name=portainer --restart=always -v /var/run/docker.sock:/var/run/docker.sock -v portainer_data:/data portainer/portainer-ce:linux-arm
```

Then go to your browser to:

http://[rpi's ip address]:9000

# Add the Docker container from Zanzivyr's Dockerhub
Create an admin login
Login

Click on the "local" environment
Click "Containers" on the left

Click "Add Container"

- Create a name with no spaces
- Add the Image "zanzivyr/ros2foxy:latest"
- Scroll down then click "Deploy the container"

# Creating a Docker image -> container
In general you can follow these instructions to make a Docker image: 

https://medium.com/codex/dockerfile-explained-dockerize-nodejs-application-411dadbc3412

Then to push this to DockerHub follow these instructions: 

https://medium.com/codex/push-docker-image-to-docker-hub-acc978c76ad

Here's how, in short:

```
mkdir ecobot && cd ecobot
touch Dockerfile
vim Dockerfile
```

Copy and paste the code from the section "Creating a Dockerfile to build ROS packages" into the Dockerfile. (I will include that code here in a section below just for completeness.) https://hub.docker.com/r/arm64v8/ros/

Now build your Docker image

`docker build -t [name]:[tag] .`
Ex: `docker build -t ros2foxy:latest .`

Make sure you login to Dockerhub first

`docker login`

Then tag it. Make sure the name and the tag are the same for both volumes.

`docker tag [name]:[tag] [dockerhub name]/[name]:[tag]`
Ex: `docker tag ros2foxy:latest zanzivyr/ros2foxy:latest`

Finally, push it to Dockerhub

`docker push [dockerhub name]/[name]:[tag]`
Ex: `docker push zanzivyr/ros2foxy:latest`

--------------------------------

# Creating a Dockerfile to build ROS packages
```
ARG FROM_IMAGE=arm64v8/ros:foxy
ARG OVERLAY_WS=/opt/ros/overlay_ws

## multi-stage for caching
FROM $FROM_IMAGE AS cacher

## clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
RUN echo "\
repositories: \n\
## change these file names to your git repo. you don't need version. also, remember these names for later.
  ros2/demos: \n\
    type: git \n\
    url: https://github.com/ros2/demos.git \n\
    version: ${ROS_DISTRO} \n\
" > ../overlay.repos
RUN vcs import ./ < ../overlay.repos

## copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

## multi-stage for building
FROM $FROM_IMAGE AS builder

## install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths \
## change these names to the structure of your repo. for example, if you wrote above "zanzivyr/directory" and in your repo the package is at "src/package", then you want to write "src/zanzivyr/directory/src/package"
        src/ros2/demos/demo_nodes_cpp \
        src/ros2/demos/demo_nodes_py \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

## build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --packages-select \
## add your package names. as with above, it should be something like "package"
        demo_nodes_cpp \
        demo_nodes_py \
      --mixin $OVERLAY_MIXINS

## source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

## run launch file
## this should mirror your ros package structure
CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]
```

