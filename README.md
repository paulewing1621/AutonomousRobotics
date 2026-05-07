# Autonomous-Robotics


## How to launch the autonomous mission:

**On the TurtleBot (SSH in first):**
```bash
ssh turtlebot@turtlebot01
```

Then run each of the following in a separate terminal on the robot:
```bash
ros2 launch turtlebot_launch minimal.launch.py
ros2 launch turtlebot_launch rplidar_a1_launch.py
ros2 launch turtlebot_launch autodock.launch.py
ros2 launch turtlebot_launch kinect_min_comp.launch.py
ros2 launch floor_nav launch_server_tb.launch.py
ros2 run topic_tools mux /velocity_smoother/input /teleop/twistCommand /mux/autoCommand --ros-args -r __name:=cmd_mux
ros2 service call /cmd_mux/select topic_tools_interfaces/srv/MuxSelect '{topic: "/mux/autoCommand"}'
ros2 launch wpa_cli wpa_cli.launch.py
```

**On the PC:**
```bash
ros2 launch turtlebot_launch slam_tb_sync.launch.py use_sim_time:=False
ros2 launch wifi_map_base wifi_map.launch.py
rviz2
ros2 run floor_nav mission_dedock_move_dock.py
```
