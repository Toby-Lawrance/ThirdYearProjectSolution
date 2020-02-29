source install/setup.bash
ros2 topic pub -1 /reset std_msgs/Empty
ros2 run object_tracking runRobot
ros2 topic pub -1 /cmd_vel geometry_msgs/Twist "[0.0, 0.0, 0.0]" "[0.0, 0.0, 0.0]"