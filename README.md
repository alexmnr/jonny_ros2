# Overview
Everything you need to controll the fully 3D-printed robotic arm "Jenny" using ros2 and moveit.

<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->

- [Installation](#installation)
  - [ROS2](#ros2)
  - [rosdep](#rosdep)
  - [Dependencies](#dependencies)
  - [Build](#build)
- [Usage](#usage)
  - [Initialization (once after reboot)](#initialization-once-after-reboot)
    - [Sourcing the workspace](#sourcing-the-workspace)
    - [Attaching the can bus to linux](#attaching-the-can-bus-to-linux)
  - [Launch (every time you want to start the programm)](#launch-every-time-you-want-to-start-the-programm)
    - [Launch real robot with gui controll](#launch-real-robot-with-gui-controll)
    - [Launch real robot with command controll](#launch-real-robot-with-command-controll)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# Installation
Follow these instructions to use this repository.

## ROS2 
To work with this repository, you need a working install of ros2 jazzy. Install it locally by following the instructions here: https://docs.ros.org/en/jazzy/Installation.html 

Alternatively, you can use the docker image from https://github.com/alexmnr/jenny_docker
First Option is **highly** recommended though.

## rosdep
To automatically install all necessary dependencies, this projects uses rosdep (https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html). 
Install Rosdep using these commands:
```sh
sudo apt install python3-rosdep
```
```sh
sudo rosdep init
rosdep update
```

## Dependencies
Now, clone this repository:
```sh
git clone https://github.com/alexmnr/jenny_ros2.git
```
Use rosdep to install all dependencies automatically:
```sh
cd jenny_ros2/ros && rosdep install --from-paths src -y --ignore-src
```

## Build
To finish the install, all packages provided have to be build.

**IMPORTANT:** This is also necessary after updating the repository or making any changes to the code.

The build command has to be run in the \<path to repository\>/ros folder.
```sh
cd jenny_ros2/ros
```
Build Command:
```sh
colcon build
```

# Usage
Ok, let's do it.
## Initialization (once after reboot)
### Sourcing the workspace
Before all the packages are ready to be used, you have to tell ros where to find them. To do that, I added a simply script to the repository.
```sh
source ./activate.sh
```
### Attaching the can bus to linux
First, attach the can bus adapter using the usb-c cable.

Then run :
```sh
sudo slcan_attach -f -s6 -o /dev/ttyACM0 && sudo slcand -o -s6 /dev/ttyACM0 can0 && sudo ip link set can0 up
```

## Launch (every time you want to start the programm)
There are multiple options available in this repository:
### Launch real robot with gui controll
```sh
ros2 launch jenny_moveit_config real_robot.launch.py
```
### Launch real robot with command controll
```sh
TODO: add this
```
