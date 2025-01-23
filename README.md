# Current Control Demo:
source /opt/ros/humble/setup.bash

cd lunabotics_ros2_ws/

source install/setup.bash

ros2 run control_pkg testcontrol.py

ros2 run control_pkg drivetrain

ros2 run control_pkg scoop

## Editing packages

Make sure to remove build artifacts and colcon build --packages-select whenever making a change to a specific package

## Pushing changes
git init
git add . 
git push -u origin main
git push
