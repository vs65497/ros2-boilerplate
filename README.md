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
## Dockerfile is included in this repo
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
--------------------------------

# Setting up MicroROS
This is a multistep process. You cannot do this without Ubuntu 20.04. This is what I did to get things working.

1. Dual boot computer. https://opensource.com/article/18/5/dual-boot-linux
2. Get QEMU. https://www.tecmint.com/install-qemu-kvm-ubuntu-create-virtual-machines/
3. Get Docker.
4. Setup ESP32.

Also, here are some documents which helped me:
- AJ's documentation: https://docs.google.com/document/d/1Ph1ytrvDHn1J7ciubE9Ibcr6nY_KtMoRd5ia6QWcSuM/edit#
- Hadabot ESP32: https://www.hadabot.com/setup-esp32-to-work-with-ros2.html
- Hadabot ESP32, CLI: https://www.hadabot.com/setup-esp32-using-cli-to-work-with-ros2.html
- Install adafruit-ampy: https://learn.adafruit.com/micropython-basics-load-files-and-run-code/install-ampy
- Hadabot GitHub: https://github.com/hadabot/hadabot_main

And I should mention that any reference to int32_publisher or w/e is in the ros2 desktop version. So if you manually install that instead of using docker then you should get that to run demos.

# Dual Boot

https://opensource.com/article/18/5/dual-boot-linux 

64 GB is probably a good amount to allocate for the Ubuntu partition.

# Get QEMU

https://www.tecmint.com/install-qemu-kvm-ubuntu-create-virtual-machines/

`egrep -c '(vmx|svm)' /proc/cpuinfo`

`kvm-ok`
(This should be okay if you're dual booting)

`sudo apt install cpu-checker -y`

`sudo apt update`

`sudo apt install qemu-kvm virt-manager virtinst libvirt-clients bridge-utils libvirt-daemon-system -y`

```
sudo systemctl enable --now libvirtd
sudo systemctl start libvirtd
```

`sudo systemctl status libvirtd`

```
sudo usermod -aG kvm $USER
sudo usermod -aG libvirt $USER
```

`sudo virt-manager`

Then follow the rest of the tutorial.

Install Ubuntu 20.04 (We are going to use ROS2 Foxy for all of this).

Allocate about 20 GB for memory. If not, it's going to be a pain in the ass as you might run out of memory installing stuff for the firmware.

By the way, if you need to reallocate memory in your dual boot partition, you can follow these tutorials.

- https://www.amyschlesener.com/posts/2020/04/increasing-ubuntu-partition/
- https://www.howtogeek.com/114503/how-to-resize-your-ubuntu-partitions/


# Get Docker
https://www.simplilearn.com/tutorials/docker-tutorial/how-to-install-docker-on-ubuntu

`sudo apt install docker.io`

`sudo snap install docker`

`docker --version`

`sudo docker run hello-world`

`sudo docker images`

`sudo docker ps -a`

`sudo docker ps`

## Install Docker Compose
You may need to run compose-up, which requires different software

```
curl -L https://github.com/docker/compose/releases/download/1.24.1/docker-compose-`uname -s`-`uname -m` -o /usr/local/bin/docker-compose
chmod +x /usr/local/bin/docker-compose
```
(You should check that this is the latest version)

`docker-compose --version`

# Setup ESP32
Now... after all of that you can setup the ESP32 with MicroROS.

https://cps.unileoben.ac.at/install-micro-ros-on-esp32/

## We'll start by making a directory for files your docker container will use.

```
cd ~
mkdir uros
```

## Bind your docker container to that directory with this command

`docker run -it --net=host -v /dev:/dev -v ~/uros:/uros --privileged ros:foxy`

## Download micro-ROS and install dependencies

```
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

Create new firmware workspace -- this time for platform esp32

(Pay attention to use platform "esp32")

`ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32`

## Configure the firmware to include the application

```
# Configure step with ping_pong app and serial transport
ros2 run micro_ros_setup configure_firmware.sh ping_pong --transport serial
```

## Build the firmware

```
# Build step
ros2 run micro_ros_setup build_firmware.sh
```

## Flash the firmware

(I did not have to do this step.)

Connect esp32 via USB cable or appropriate interface. Most likely all the esp32 dev-boards offer USB connection and will be recognized as ttyUSB0 (USB serial device). In this step, the flash program will find the right port for you automatically.

Caveat ahead: If you are not running the process as root user (which is the recommended approach anyways), you must make sure that you have write permissions on the /dev/ttyUSB0 device. That can be established like follows:

`sudo chown $USER /dev/ttyUSB0`

### Make sure you connect the USB port to the VM.
- Click "Virtual Machine" at the top then choose "Redirect USB device".
- Check the box which says "Silicon Labs CP2102 USB to UART Bridge Controller ..."
- If you don't see this, you can find your ESP32 on the home system by doing the following:

```
lsusb
ls -la /dev/serial/by-id/
```

Depending on your configuration, it is possible that you need to adapt above command. By running the flash step you will most likely see which device you need to use when the process exits with an error 🙂

```
# Flash step
ros2 run micro_ros_setup flash_firmware.sh
```

## Connect to the device and test the firmware

### MAKE SURE YOU PRESS THE "EN" BUTTON

To make sure everything is working, disconnect the device after the flashing process is done or just reset it via the enable button (EN).

```
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
```

Replace [device] with your serial connection. Again, remember to choose the /dev/ttyUSB* device selected in the flash step. To avoid searching for the correct serial port in the future, you can also find out the discrete device identifier by having a look at this command’s output:

`ls /dev/serial/by-id/*`

## Testing the micro-ROS app

At this point, the micro-ROS app is built and flashed and the board is connected to a micro-ROS agent. We now want to check that everything is working.

Open a new command line. We are going to listen to the ping topic with ROS 2 to check whether the micro-ROS Ping Pong node is correctly publishing the expected pings:

### Opening a new command line
- Press ctrl-z
- Type `bg` to run the process in the background
- If you want to kill:
  - Type `ps` to find out the process ID
  - Type `kill [process id]`

```
source /opt/ros/$ROS_DISTRO/setup.bash

# Subscribe to micro-ROS ping topic
ros2 topic echo /microROS/ping
```

You should see the topic messages published by the Ping Pong node every 5 seconds:

```
user@user:~$ ros2 topic echo /microROS/ping
stamp:
  sec: 20
  nanosec: 867000000
frame_id: '1344887256_1085377743'
---
stamp:
  sec: 25
  nanosec: 942000000
frame_id: '730417256_1085377743'
---
```

At this point, we know that our micro-ROS app is publishing pings. Let’s check if it also answers to someone else’s pings. If this works, it’ll publish a pong.

So, first of all, let’s subscribe with ROS 2 to the pong topic from a new shell (notice that initially we don’t expect to receive any pong, since none has been sent yet):

```
source /opt/ros/$ROS_DISTRO/setup.bash

# Subscribe to micro-ROS pong topic
ros2 topic echo /microROS/pong
```

And now, let’s publish a fake_ping with ROS 2 from yet another command line:

```
source /opt/ros/$ROS_DISTRO/setup.bash

# Send a fake ping
ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}'
```

Now, we should see this fake_ping in the ping subscriber console, along with the board’s pings:

```
user@user:~$ ros2 topic echo /microROS/ping
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
stamp:
  sec: 305
  nanosec: 973000000
frame_id: '451230256_1085377743'
---
stamp:
  sec: 310
  nanosec: 957000000
frame_id: '2084670932_1085377743'
---
```

Also, we expect that, because of having received the fake_ping, the micro-ROS pong publisher will answer with a pong. As a consequence, in the pong subscriber console, we should see the board’s answer to our fake_ping:

(I was unable to get this to run because I was running things as background processes.)

```
user@user:~$ ros2 topic echo /microROS/pong
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
--
```

Source: https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/

