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
### Controlling the robot arm by joint_state_publisher
## Usage

### 1. Controlling the robot arm with joint_state_publisher
In the joint_state_publisher GUI, you can move the sliders to change the arm’s joint angles in real time, and see the motion directly in RViz.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch arduinobot_description display.launch.py
```
![joint_state_publisher](joint_state_publisher.jpg)


### 2. Planning with MoveIt (Kinematics / IK)
In MoveIt with RViz, you move the interactive markers to set the target pose.
Click "Planning" → MoveIt calculates the path using IK.
Preview the motion in RViz.

![plan](plan.gif)

### 3. MoveIt with Gazebo (Simulation + Planning)
You can run Gazebo together with MoveIt to simulate and plan robot arm trajectories:
![sim](sim.gif)

```cpp
# Terminal 1 - Launch Gazebo with the robot arm
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch arduinobot_mc gazebo.launch.py
```

```cpp
# Terminal 2 - Start MoveIt (planning context)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch arduinobot_mc move_group.launch.py
```

```cpp
# Terminal 3 - Open RViz with MoveIt plugin
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch arduinobot_mc moveit_rviz.launch.py
```
- Gazebo will simulate the physics of the robot arm.
- RViz (with MoveIt) allows you to set target poses, plan, and execute them.
- When you Plan + Execute in RViz, the motion is carried out inside Gazebo.


## 4. Hardware Execution (Arduino + Servo)
This section details the complete process for controlling the physical robot arm with ROS2, from programming the Arduino to running the live system.

### Section 1: Arduino Setup - The Brain of the Arm

The first and most critical step is to program the Arduino board, which will receive commands from ROS2 and translate them into physical motor movements.

#### 1.1 Finding and Uploading the Arduino Code

The firmware for the Arduino is included within this project's repository.

1.  **Locate the Firmware:** You can find the code at the following path within your workspace:
    `~/ros2_ws/src/Robot_Arm_ROS2/arduino_firmware/arduino_firmware.ino`

2.  **Upload to Arduino:**
    *   Connect your Arduino board to your computer (or pass the USB device through to your Virtual Machine).
    *   Open the `.ino` file with the Arduino IDE.
    *   Go to `Tools` -> `Board` and select your Arduino model.
    *   Go to `Tools` -> `Port` and select the serial port for your Arduino.
    *   Click the "Upload" button.



### Section 2: Running the Live Control System

To control the physical robot arm while visualizing its state in RViz, you will need to run processes in two separate terminals.

#### 2.1 Find Your Arduino's Device Name

Before launching the interface, you must identify the specific port name your system has assigned to the Arduino. This is crucial for the connection to succeed.

*   **In your terminal (inside your Linux or Virtual Machine environment), run the following command:**
    ```bash
    ls /dev/tty*
    ```
*   **Look through the output list.** Your Arduino will be listed as either `/dev/ttyUSB0` or `/dev/ttyACM0`. Note down the exact name.

#### 2.2 Launch the System

Now, open two terminals and run the following commands.

**Terminal 1: Launch the Visualization and Control GUI**

This terminal runs RViz for the 3D simulation and the `joint_state_publisher_gui` for manual control with sliders.

```bash
# Source your workspace to activate the ROS2 packages
source ~/ros2_ws/install/setup.bash

# Launch the visualization and GUI
ros2 launch arduinobot_description display.launch.py
```
**Terminal 2: Launch the Hardware Interface Node**
This terminal runs the Python script that acts as a bridge, sending commands from the GUI to the Arduino.
