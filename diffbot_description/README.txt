

1. try ros2 control with real hardware test
 - Terminal 1 : ros2 launch diffbot_description diffbot_control_rviz.launch.py
 - Terminal 2 : ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.8}, angular: {z: 0.8}}"




 



