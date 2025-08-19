# Joint State and Kinematics Control Task

Controlling the robot arm in ROS2 either manually using joint_state_publisher or automatically using MoveIt with kinematics, in both simulation and hardware.

## Project Requirements

- ROS2 Humble on Ubuntu 22.04
- Simulation environment (RViz / Gazebo)
- Arduino + Servo motor for hardware execution

## Installation and Setup
Follow these steps to set up the project environment and dependencies.

### 1. Create a new workspace
```cpp
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

```cpp
git clone https://github.com/smart-methods/Robot_Arm_ROS2.git
```
> Note 1: make sure you already have git installed before cloning. If not:
```cpp
sudo apt update && sudo apt install -y git
```
> Note 2: after cloning, verify the package was downloaded:
```cpp
ls -la src/
```
You should see a folder named Robot_Arm_ROS2 inside ~/ros2_ws/src.

### 2. Install dependencies
```cpp
sudo apt-get update
sudo apt-get install -y \
  ros-humble-joint-state-publisher-gui \
  ros-humble-gazebo-ros \
  ros-humble-xacro \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-controller-manager \
  ros-humble-moveit \
  ros-humble-gazebo-ros2-control
```

### 3. Build the workspace
```cpp
colcon build --symlink-install
```

## Usage
