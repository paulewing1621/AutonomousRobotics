#!/usr/bin/env python3
import sys
import rclpy
from task_manager_client_py.TaskClient import *
 
rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)
 
try:
    tc.AutoDock()
 
except TaskException as e:
    tc.get_logger().error("Exception caught: " + str(e))
 
tc.get_logger().info("Mission completed")