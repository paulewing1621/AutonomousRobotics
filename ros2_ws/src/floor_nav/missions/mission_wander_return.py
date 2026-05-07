#!/usr/bin/env python3
import sys
import math
import time
import rclpy
import rclpy.time
import tf2_ros
from task_manager_client_py.TaskClient import *

RETURN_TIMEOUT = 120.0

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)

print("Waiting 5 seconds before starting mission...")
time.sleep(5.0)

try:
    print("Step 1: Moving backward...")
    tc.Constant(linear=-0.2, angular=0.0, duration=3.0)

    print("Step 2: Capturing dock position from TF (map→base_link)...")
    _tmp = rclpy.create_node('dock_pos_reader')
    _buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(_buf, _tmp)
    _deadline = time.time() + 30.0
    _last_exc = None
    dock_x = dock_y = None
    while time.time() < _deadline:
        rclpy.spin_once(_tmp, timeout_sec=0.2)
        try:
            _t = _buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            dock_x = _t.transform.translation.x
            dock_y = _t.transform.translation.y
            break
        except Exception as _e:
            _last_exc = _e
    _tmp.destroy_node()

    if dock_x is None:
        tc.get_logger().error("TF map→base_link indisponible après 30s : %s" % _last_exc)
        rclpy.shutdown()
        sys.exit(1)

    tc.get_logger().info("Position dock capturée : (%.3f, %.3f)" % (dock_x, dock_y))

    print("Step 3: Wandering for 10 seconds...")
    tc.Wander(
        max_linear_speed=0.2,
        safety_range=0.5,
        dont_care_range=1.5,
        max_angular_speed=0.5,
        multiplier=0.3,
        front_sector=True,
        angular_range=math.pi / 6,
        task_timeout=10.0,
    )

    print("Step 4: Returning to dock position (x=%.3f, y=%.3f)..." % (dock_x, dock_y))
    tc.PlanTo(
        goal_x=dock_x,
        goal_y=dock_y,
        dist_threshold=0.5,
        task_timeout=RETURN_TIMEOUT,
        pub_period=1.0,
    )
    tc.get_logger().info("Retour base réussi.")

    print("Mission completed! Robot returned to dock position.")

except TaskException as e:
    tc.get_logger().error("Exception caught: " + str(e))
