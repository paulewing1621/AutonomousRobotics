#!/usr/bin/env python3
import sys
import rclpy
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)

try:
    print("Moving forward...")
    tc.Constant(linear=0.2, angular=0.0, duration=3.0)

    print("Moving backward...")
    tc.Constant(linear=-0.2, angular=0.0, duration=3.0)

    print("Mission completed!")

except TaskException as e:
    tc.get_logger().error("Exception caught: " + str(e))
