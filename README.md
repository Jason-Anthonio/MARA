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

# check the controllers:
ros2 control list_controllers
## make sure they're all active

# using the position
ros2 topic pub --once /position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [1.5708, 0.0, 0.0, 0.0, 0.0]}"
waist, shoulder, elbow, wrist, gripper

1.5708 is 90 degrees angle
0.7854 is 45 degrees angle
gripper is maxed at 0.04 for closed grip

# using the velocity
## disabling the position controller
ros2 control switch_controllers --deactivate position_controller

## running
ros2 topic pub -r 10 /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.2, 0.0, 0.0, 0.0, 0.0]}"

## stopping
ros2 topic pub --once /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0]}"