# arm\_description (MARA 5-DOF Robotic Arm \- ROS 2 Jazzy)

`arm_description` is the upgraded **ROS 2 Jazzy** robot description package for the **MARA (Modular Adaptive Robotic Arm)** 5-DOF anthropomorphic assistive robotic arm.

---

## 1\. Overview & Conversion Summary

This package upgrades the original ROS 1 `ASSY_description` package to full **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic** compatibility.

### Summary of Converted Components

| File / Component | Original (ROS 1\) | Converted Version (ROS 2 Jazzy) | Description of Changes |
| :---- | :---- | :---- | :---- |
| `package.xml` | Format 2 (catkin) | Format 3 (`ament_cmake`) | Updated buildtool to `ament_cmake`, replaced ROS 1 dependencies (`rviz`, `joint_state_controller`, `gazebo_ros_control`) with ROS 2 counterparts (`rviz2`, `ros2_control`, `ros2_controllers`, `gz_ros2_control`, `ros_gz_sim`, `ros_gz_bridge`, `joint_state_broadcaster`, `joint_trajectory_controller`). |
| `CMakeLists.txt` | `catkin_package()` / `find_package(catkin)` | `find_package(ament_cmake REQUIRED)` / `install(DIRECTORY ...)` | Modernized cmake version to 3.8, added `ament_package()`, directory installation for `config/`, `launch/`, `meshes/`, and `urdf/` to `share/${PROJECT_NAME}`. |
| `launch/display.launch.py` | ROS 1 XML `display.launch` | Python Launch (`display.launch.py`) | Uses `launch_ros` to process Xacro via `Command`, launches `robot_state_publisher`, `joint_state_publisher_gui` (or `joint_state_publisher`), and `rviz2` with `-d` config. |
| `launch/gazebo.launch.py` | ROS 1 XML `gazebo.launch` (Gazebo Classic) | Python Launch (`gazebo.launch.py`) (Gazebo Harmonic / `ros_gz_sim`) | Launches `ros_gz_sim`, spawns entity from `/robot_description` topic using `create` node, runs `ros_gz_bridge` for `/clock` and `/joint_states`, and auto-spawns `ros2_control` controllers. |
| `launch/controller.launch.py` | ROS 1 XML `controller.launch` | Python Launch (`controller.launch.py`) | Spawns `joint_state_broadcaster` and `arm_controller` using `controller_manager/spawner`. |
| `config/controller.yaml` | ROS 1 `joint_state_controller` / `effort_controllers` | ROS 2 `controller_manager` configuration | Configured `joint_state_broadcaster`, `joint_trajectory_controller`, and `forward_command_controller` for `joint_1` through `joint_5`. |
| `launch/urdf.rviz` | RViz 1 config | RViz 2 Displays Config | Updated display plugins to `rviz_default_plugins/RobotModel`, `rviz_default_plugins/Grid`, and `rviz_default_plugins/TF` with `base_link` fixed frame. |
| `urdf/ASSY.xacro` | `package://ASSY_description/...` | `package://arm_description/...` | Updated mesh URLs, standard joint names (`joint_1` to `joint_5`), link lengths (Shoulder $a\_1 \= 25\\text{ cm}$, Elbow $a\_2 \= 19\\text{ cm}$, Wrist $a\_3 \= 6\\text{ cm}$), and modular includes. |
| `urdf/ASSY.trans` | ROS 1 `<transmission>` tags | `<ros2_control name="ArmSystem" type="system">` | Configured `gz_ros2_control/GazeboSimSystem` hardware plugin with position command and position/velocity state interfaces for all 5 revolute joints. |
| `urdf/ASSY.gazebo` | `libgazebo_ros_control.so` | `gz_ros2_control-system` | Configured `gz_ros2_control::GazeboSimROS2ControlPlugin` pointing to `$(find arm_description)/config/controller.yaml` and physical link friction/material properties. |
| `urdf/materials.xacro` | Color macros | ROS 2 Xacro materials | Standardized RGBA color definitions (`silver`, `black`, `blue`, `orange`, `white`). |

---

## 2\. Directory Structure

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
