#!/bin/bash

delay="$1"
ros2 topic pub -1 /commands/led1 kobuki_ros_interfaces/msg/Led "{value: 0}"
if test "x$delay" != "x"
then
	sleep $delay
fi


ros2 topic pub -1 /commands/led1 kobuki_ros_interfaces/msg/Led "{value: 1}"
ros2 bag record /aruco_detections /commands/velocity /kinect/camera_info /kinect/depth/camera_info /kinect/image_raw_throttle \
	/odom /robot_description /scan /sensors/battery_state /sensors/core /sensors/imu_data /tf /tf_static /landmark /wpa_cli/scan
ros2 topic pub -1 /commands/led1 kobuki_ros_interfaces/msg/Led "{value: 0}"
