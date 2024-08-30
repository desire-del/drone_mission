# Autonomous UAV for ship inspection

## Requirements

### System Requirements

* [ROS Humble](https://docs.ros.org/en/humble/Installation.html)

* [Gazebo Garden](https://gazebosim.org/docs/garden/install)

* [Cartographer ROS](https://google-cartographer-ros.readthedocs.io/en/latest/)

* [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)


### Workspace Requirements

* [Ardupilot](https://github.com/ArduPilot/ardupilot_gz)

* [Ai Coper](https://github.com/desire-del/ai_copter)

* [Ros 2]()

* [Gazebo]()

## Installation

### Setup workspace
Follow this youtube tutorial for setup the workspace: [youtube](https://www.youtube.com/watch?v=2BhyKyzKAbM&ab_channel=XiaodiTao)

### Install ai_copter package

```bash
cd ~/ros2_ws/src
git clone https://github.com/desire-del/ai_copter.git

```
### Install yolov8_ros
```bash
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/yolov8_ros.git
pip3 install -r yolov8_ros/requirements.txt
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build

```
### Install drone_mission
```bash
cd ~/ros2_ws/src
git clone https://github.com/desire-del/drone_mission.git

```
## Build

Build it with colcon build:
```bash
cd ~/ros2_ws
colcon build

```

## Usage

### 1. Launch Gazebo Rviz and Ardupilot

This simulation has an Iris copter equipped with camera in a port world.
To launch rviz gazebo and ardupilot, run:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch drone_mission drone_mission.launch.py
```
In another terminal launch ship detection node 
si vous avez un GPU disponible 
```shell
ros2 launch yolov8_bringup yolov8.launch.py
```
sinon 
```shell
ros2 launch yolov8_bringup yolov8.launch.py device:=cpu
```
In another terminal, with the world and copter in place, Mavproxy:

```bash
cd ~/ros2_ws
source install/setup.bash
mavproxy.py --console --aircraft test --master :14550

```
