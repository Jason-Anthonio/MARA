# arm\_description (MARA 5-DOF Robotic Arm \- ROS 2 Jazzy)

`arm_description` is the robot description package for **MARA (Modular Adaptive Robotic Arm)** 5-DOF anthropomorphic assistive robotic arm.

---

## 1\. Directory Structure

    arm\_description/
    ├── CMakeLists.txt
    ├── LICENSE
    ├── package.xml
    ├── README.md
    ├── config/
    │   └── controller.yaml
    ├── launch/
    │   ├── controller.launch.py
    │   ├── controller.yaml
    │   ├── display.launch.py
    │   ├── gazebo.launch.py
    │   └── urdf.rviz
    ├── meshes/
    │   ├── 2nd link.stl
    │   ├── 2nd\_link\_1.stl
    │   ├── 3rd link.stl
    │   ├── 3rd\_link\_1.stl
    │   ├── 4th link.stl
    │   ├── 4th\_link\_1.stl
    │   ├── 5th link.stl
    │   ├── 5th\_link\_1.stl
    │   └── base\_link.stl
    └── urdf/
    ├── arm.urdf.xacro
    ├── ASSY.gazebo
    ├── ASSY.trans
    ├── ASSY.xacro
    ├── gazebo.xacro
    ├── materials.xacro
    └── ros2\_control.xacro

---

## 3\. Kinematic Architecture & Specifications

The MARA robotic arm features a 5-DOF articulated chain:

1. **Joint 1 (`joint_1`)**: Base rotation (Yaw, Axis: Z, Limits: $\\pm 180^\\circ$ / $\[-3.1416, 3.1416\]\\text{ rad}$)  
2. **Joint 2 (`joint_2`)**: Shoulder elevation (Pitch, Axis: Y, Limits: $\\pm 90^\\circ$ / $\[-1.5708, 1.5708\]\\text{ rad}$, Link length $a\_1 \= 25.0\\text{ cm}$)  
3. **Joint 3 (`joint_3`)**: Elbow flex/extension (Pitch, Axis: Y, Limits: $\\pm 90^\\circ$ / $\[-1.5708, 1.5708\]\\text{ rad}$, Link length $a\_2 \= 19.0\\text{ cm}$)  
4. **Joint 4 (`joint_4`)**: Wrist pitch (Pitch, Axis: Y, Limits: $\\pm 180^\\circ$ / $\[-3.1416, 3.1416\]\\text{ rad}$, Link length $a\_3 \= 6.0\\text{ cm}$)  
5. **Joint 5 (`joint_5`)**: Tool roll / End-effector mount (Roll, Axis: Z, Limits: $\\pm 90^\\circ$ / $\[-1.5708, 1.5708\]\\text{ rad}$)

---

## 4\. Build & Installation

### Prerequisites

Ensure you have ROS 2 Jazzy and the required dependencies installed:

    sudo apt update
    sudo apt install \-y \\
      ros-jazzy-robot-state-publisher \\
      ros-jazzy-joint-state-publisher \\
      ros-jazzy-joint-state-publisher-gui \\
      ros-jazzy-rviz2 \\
      ros-jazzy-xacro \\
      ros-jazzy-ros2-control \\
      ros-jazzy-ros2-controllers \\
      ros-jazzy-gz-ros2-control \\
      ros-jazzy-ros-gz-sim \\
      ros-jazzy-ros-gz-bridge

### Build with Colcon

From your ROS 2 workspace (e.g., `~/ros2_ws` or `/ws_MARA`):

    cd \~/ros2\_ws
    
    colcon build \--symlink-install \--packages-select arm\_description
    
    source install/setup.bash

---

## 5\. Usage & Launch Instructions

### A. Visualize in RViz2 with Joint State Slider GUI

To inspect the robot model, joints, and TF frames:

    ros2 launch arm\_description display.launch.py


### B. Launch in Gazebo Simulation (Gazebo Harmonic)

To spawn the arm into a Gazebo physics environment with active `ros2_control` hardware simulation:

    ros2 launch arm\_description gazebo.launch.py


### C. Hardware Interface & Controller Bringup

When running on real robot hardware (Raspberry Pi 5 / Workstation with serial/microcontroller interface):

    ros2 launch arm\_description controller.launch.py  
