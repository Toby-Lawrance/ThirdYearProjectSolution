exec >> "running.log" 2> "error.log"

source install/setup.bash

ros2 launch turtlebot3_bringup robot.launch.py &

#ros2 run object_tracking runRobot &