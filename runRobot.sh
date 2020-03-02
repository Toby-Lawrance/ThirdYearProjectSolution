exec >> "run.log" 2> "error.log"
#Output nicely to files
source install/setup.bash
ros2 run object_tracking runRobot
#It's like JSON?
ros2 topic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0,y: 0,z: 0}}"
