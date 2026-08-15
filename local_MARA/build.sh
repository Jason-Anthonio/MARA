#error out
set -e

#Env
source /opt/ros/jazzy/setup.bash
source MarAIenv/bin/activate

#Clean and Build
rm -rf build install log
colcon build

source install/setup.bash

