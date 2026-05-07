# Autonomous-Robotics
CS-7630 Autonomous Robotics Course


## How to launch collision avoidance on the turtlebot :

**Terminal 1:**
- tb1
- zenod

**Terminal 2: ROBOT**
- tb1
- ssh turtlebot@turtlebot01
- byobu
- ros2 launch turtlebot_launch minimal.launch.py

**Terminal 3: ROBOT**
- ros2 launch turtlebot_launch rplidar_a1_launch.py

//**Terminal 4:**
//- source install/setup.bash
//- tb1
//- ros2 launch vrep_ros_teleop teleop_joy_only.launch.py

**Terminal 5:**
- tb1
- source install/setup.bash
- ros2 launch collision_avoidance_base ca_turtle.launch.py scan_topic:=/scan

**Final Project**
ros2 launch floor_nav autonomous_mission.launch.py use_sim_time:=False