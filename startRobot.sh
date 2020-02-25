exec >> "run.log" 2> "error.log"
#Output nicely to files
source install/setup.bash

ros2 launch turtlebot3_bringup robot.launch.py &

#ros2 run object_tracking runRobot &
