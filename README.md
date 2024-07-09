# Guid to implement a simple pick and place program for UR10e leveraging the Cartesian position control and Joint Position Control

This repository contains a Catkin workspace for controlling the UR10e robot in a Gazebo simulation using ROS. It includes scripts for running simple pick and place task by using the joint position control and Cartesian position control.

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
git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
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
roslaunch ur_gazebo ur10e_bringup.launch
```
2. **Run MoveIt!**
```
roslaunch ur10e_moveit_config moveit_planning_execution.launch sim:=true
roslaunch ur10e_moveit_config moveit_rviz.launch config:=true
```
3. **Run the Joint Position Control Script**
```
rosrun joint_control joint_pick_and_place_cube_position.py
```
Or
```
cd catkin_ws/src/joint_control/scripts
python3 joint_pick_and_place_cube_position.py
```
This script control the robot by given joint angle values. It moves the UR10e to pick up position and after picking the object moves the robot to drop off zone. finally when the task is completed, the robot returns to the home position. As well it is publishing the cube's position and "up" vector every 500 ms.

4. **Run the Cartesian Position Control Script**
```
rosrun cartesian_control cartesian_position_control.py
```
Or
```
cd catkin_ws/src/cartesian_control/scripts
python3 cartesian_position_control.py
```
This script control the robot by given cartesian positions. It moves the UR10e to pick up position and after picking the object moves the robot to drop off zone. finally when the task is completed, the robot returns to the home position. As well it is publishing the cube's position and "up" vector every 500 ms.

# Extra attached documents

The Design Proposal for the Pick and Place Solution_Vention.pdf file contains the design Proposal and the justification of robot selection, gripper selection and calibration procedure.
The "cartesian_position_control_pick_and_place.mp4" file is a short video of the robots pick and place task in gazebo simulation leveraging the cartesian position control.
The "joint_position_control_pick_and_place.mp4" file is a short video of the robots pick and place task in gazebo simulation leveraging the joint position control.
The "published_cartesian_positions.png" and "published_joint_positions.png" are two screenshots of the terminal from requested output positions.
