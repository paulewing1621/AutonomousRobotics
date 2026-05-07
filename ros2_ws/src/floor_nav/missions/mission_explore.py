#!/usr/bin/env python3
import sys
import time
import signal
import os
import subprocess
import rclpy
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)

EXPLORE_DURATION = 30.0

ca_proc = None

try:
    print("Starting exploration (ca_turtle.launch)...")
    ca_proc = subprocess.Popen(
        ['ros2', 'launch', 'collision_avoidance_base', 'ca_turtle.launch.py'],
        start_new_session=True
    )

    print(f"Exploring for {EXPLORE_DURATION} seconds...")
    time.sleep(EXPLORE_DURATION)

    print("Stopping exploration...")
    os.killpg(ca_proc.pid, signal.SIGTERM)
    ca_proc.wait()
    ca_proc = None

    print("Mission completed!")

except TaskException as e:
    tc.get_logger().error("Exception caught: " + str(e))

finally:
    if ca_proc is not None:
        try:
            os.killpg(ca_proc.pid, signal.SIGTERM)
            ca_proc.wait()
        except Exception:
            pass
