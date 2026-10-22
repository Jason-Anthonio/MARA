# MARA
MARA (Modular/Adaptive Robot Arm) is an AI-powered Cobot primarily focued on casual indoor applications


# how to use the digital twin:
go to the mara description
run:
## for colcon vv
bash build.sh 
## then
source install/setup.bash
ros2 launch marav2ckptbest_description display.launch.py

# Gazebo simulation (arm + stereo cameras)
Runs the arm in Gazebo Harmonic with gz_ros2_control and both wrist cameras.
On start, the arm moves to a "scan" pose that looks at three test objects.
```
cd ws_MARA
colcon build --packages-select marav3best_description mara_gazebo
source install/setup.bash
ros2 launch mara_gazebo sim.launch.py              # Gazebo GUI + RViz with both camera feeds
ros2 launch mara_gazebo sim.launch.py gui:=false   # headless Gazebo
```
The same trajectory commands as below work in the sim.
The camera images are published on /camera_top/image_raw and /camera_bottom/image_raw (plus camera_info).
On a hybrid-graphics laptop, Gazebo must render on the NVIDIA GPU (check `prime-select query`).

# check the controllers:
ros2 control list_controllers
## make sure they're all active

# using a smooth trajectory
ros2 topic pub --once /trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [Revolute_1, Revolute_5, Revolute_7, Revolute_9, Slider_10], points: [{positions: [1.5708, 0.0, 0.0, 0.0, 0.02], time_from_start: {sec: 3, nanosec: 0}}]}"

# using the direct position command without step animation
## currently replaced by joint trajectory
ros2 topic pub --once /position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [1.5708, 0.0, 0.0, 0.0, 0.0]}"
waist, shoulder, elbow, wrist, gripper

1.5708 is 90 degrees angle
0.7854 is 45 degrees angle
gripper is maxed at 0.04 for closed grip

# using the velocity
## disabling the position controller
ros2 control switch_controllers --deactivate trajectory_controller

## running
ros2 topic pub -r 10 /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.2, 0.0, 0.0, 0.0, 0.0]}"

## stopping
ros2 topic pub --once /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0]}"