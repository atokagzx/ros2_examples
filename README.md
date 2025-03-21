# ROS2 workspace
This repository contains ROS2 workspace with examples specially designed for MISIS Robotics course.

## Table of contents
- [How to run](#how-to-run)
- [How to use](#how-to-use)

## How to run
0. Install Docker from [official site](https://docs.docker.com/engine/install/ubuntu/)  
**Note:** Remember to run *post-installation* steps
1. Clone this repository to your ROS2 workspace
```bash
git clone -b develop https://github.com/atokagzx/ros2_examples.git
cd ros2_examples
```
2. Run the following command to run container
```bash
./scripts/run_docker.sh
```
3. Run the following command **inside the container** to build the workspace
```bash
colcon build
source install/setup.zsh
```
4. Run the following command to attach to the container
```bash
./scripts/attach_docker.sh
```

## How to use
- There is [**ros_ws**](/ros_ws) directory for your packages that is mounted to the container at `/root/ros_ws` path. It is  linked to the workspace so you can edit your packages there and do not need to rerun the container.
- Start scripts starts X server to run GUI applications. You can run GUI applications like rviz, rqt, etc. inside the container.
