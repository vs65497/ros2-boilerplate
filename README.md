# ros2-boilerplate
Boilerplate for ros2 enabled platforms
Last updated: April 1, 2023

# Setting up Raspberry Pi
Using Raspberry Pi Imager, select:

Ubuntu Server 22.04.2 LTS (64-bit) ARM x64
Set hostname: "____.local"
Enable SSH, Use password authentication
Set username and password
Configure wireless LAN
SSID and Password: [wifi credentials]
Wireless LAN country: US
Set locale settings: America, us
Play sound when finished, Eject media when finished, Enable telemetry

In order to connect the RPi to the network it was necessary to connect an ethernet cable to the pi on first initialization.

If you go to the router webpage, you should be able to see the new pi come online under the wireless section. If all goes well then the hostname of the pi will also be updated.

This didn't work every time. I just reflashed the SD card and tried again with the pi connected directly to the router.

Take note of the IP address. I was typically not able to find the pi with a normal scan of the router network.

nmap -sn 192.168.1.0/24

To connect:

ssh [username]@[ip address]

Usually I make a shell script for this saved as [username].sh just so I don't forget the IP address.

# Install ROS2 Humble
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt upgrade

sudo apt install ros-humble-ros-base

sudo apt install ros-dev-tools

At this point it may be necessary to restart the RPi.

sudo reboot

sudo apt install gh

gh auth login

*Here, you will need to have zanzivyr give permissions to ssh connect to the repo. This repo needs to be moved to a public location.

# Install Boilerplate
gh repo clone zanzivyr/ros2-boilerplate

cd ~/ros2-boilerplate

Source the workspace

. install/setup.bash

colcon build --symlink-install

# Running the Boilerplate
In the current window run the following:

Source the workspace.

. install/setup.bash

Run.

ros2 run cpp_pubsub talker

Open a new window then run the following:

cd ~/ros2-boilerplate

. install/setup.bash

ros2 run cpp_pubsub listener

# For running the Launch file
cd ~/ros2-boilerplate

. install/setup.bash

ros2 launch cpp_pubsub startup.py