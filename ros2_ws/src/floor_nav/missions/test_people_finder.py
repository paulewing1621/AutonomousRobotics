#!/usr/bin/env python3

import sys
import rclpy
from math import pi
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)

try:
    while rclpy.ok():
        tc.get_logger().info("Phase 1")
        wander_id = tc.Wander(foreground=False)
        tc.WaitForFace(foreground=True)
        tc.get_logger().info("Face Detected")
        tc.stopTask(wander_id)
        tc.get_logger().info("Phase 2")
        stare_id = tc.StareAtFace(foreground=False)
        tc.Wait(duration=5.0)
        tc.stopTask(stare_id)
        tc.get_logger().info("Phase 3")
        tc.SetHeading(target=1.57, relative=True)
except TaskException as e:
    tc.get_logger().error("Exception caught: " + str(e))

tc.get_logger().info("Mission stopped")
