# Catkin Workspace for UR10e Cartesian and Joint Position Control

This repository contains a Catkin workspace for controlling the UR10e robot in a Gazebo simulation using ROS. It includes scripts for joint position control and Cartesian position control.

## Setup Instructions

### Prerequisites

- Ubuntu 20.04 LTS
- ROS Noetic

### Installation

1. **Install ROS Noetic**:
   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt install curl
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   sudo rosdep init
   rosdep update
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

2. **Clone this Repository**
```
cd ~
git clone https://github.com/your-username/catkin_ws.git
cd catkin_ws
```

3. **Install Dependencies**
```
rosdep install --from-paths src --ignore-src -r -y
```
4. **Build the Workspace**
```
catkin_make
source devel/setup.bash
```
# Usage

1. **Run the Gazebo Simulation**
```
roslaunch ur_gazebo ur10e.launch
```
2. **Run MoveIt!**
```
roslaunch ur10e_moveit_config ur10e_moveit_planning_execution.launch
roslaunch ur10e_moveit_config moveit_rviz.launch config:=true
```
3. **Run the Joint Position Control Script**
```
rosrun joint_control joint_position_control.py
```
4. **Run the Cartesian Position Control Script**
rosrun cartesian_control cartesian_position_control.py


