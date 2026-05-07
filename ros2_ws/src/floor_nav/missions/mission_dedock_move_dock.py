#!/usr/bin/env python3
import sys
import time
import signal
import math
import os
import subprocess
import rclpy
import rclpy.time
import tf2_ros
from task_manager_client_py.TaskClient import *
from std_srvs.srv import SetBool
from sensor_msgs.msg import BatteryState

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)

_battery_pct = None
_battery_node = rclpy.create_node('battery_monitor')
def _on_battery(msg: BatteryState):
    global _battery_pct
    _battery_pct = msg.percentage
_battery_node.create_subscription(BatteryState, '/sensors/battery_state', _on_battery, 1)

EXPLORE_DURATION = 240.0  # seconds

print("Waiting 5 seconds before starting mission...")
time.sleep(5.0)

ca_proc = None

def set_explorer_enabled(enable):
    request = SetBool.Request()
    request.data = enable

    while not tc.enable_explorer_client.wait_for_service(timeout_sec=1.0):
        tc.get_logger().info('Waiting for /occgrid_planner/enable_explorer service...')

    future = tc.enable_explorer_client.call_async(request)
    rclpy.spin_until_future_complete(tc, future)
    response = future.result()

    if response is None:
        raise RuntimeError('enable_explorer service call returned no response')

    if not response.success:
        raise RuntimeError(f'enable_explorer service failed: {response.message}')

    tc.get_logger().info(response.message)

tc.enable_explorer_client = tc.create_client(SetBool, '/occgrid_planner/enable_explorer')

try:
    print("Step 1: Undocking (moving backward)...")
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
            _t = _buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            dock_x = _t.transform.translation.x
            dock_y = _t.transform.translation.y
            q = _t.transform.rotation
            dock_theta = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
            break
        except Exception as _e:
            _last_exc = _e
    _tmp.destroy_node()

    if dock_x is None:
        tc.get_logger().error("TF map→base_link indisponible après 30s : %s" % _last_exc)
        rclpy.shutdown()
        sys.exit(1)

    tc.get_logger().info("Position dock capturée : (%.3f, %.3f, %.3f)" % (dock_x, dock_y, dock_theta))

    print("Step 3: Turning ~180 degrees...")
    tc.Constant(linear=0.0, angular=0.5, duration=11.0)
    tc.Constant(linear=0.2, angular=0.0, duration=1.0)

    rclpy.spin_once(_battery_node, timeout_sec=2.0)
    tc.get_logger().info("BATTERY : %.1f%%" % _battery_pct)

    print("Step 4: Starting exploration (ca_turtle.launch)...")
    ca_proc = subprocess.Popen(
        ['ros2', 'launch', 'collision_avoidance_base', 'ca_turtle.launch.py'],
        start_new_session=True
    )
    set_explorer_enabled(True)

    print(f"Step 5: Exploring for {EXPLORE_DURATION} seconds...")
    time.sleep(EXPLORE_DURATION)
    set_explorer_enabled(False)

    time.sleep(1.5)

    print("Step 6: Returning to dock position (x=%.3f, y=%.3f)..." % (dock_x, dock_y))
    tc.GoToPose(
        goal_x=dock_x,
        goal_y=dock_y,
        goal_theta=dock_theta,
        smart=True,
        max_velocity=0.15,
        dist_threshold=0.3,
        angle_threshold=0.3,
        k_v=1.5,
        k_alpha=2.0,
        k_beta=-1.5,
    )

    print("Step 7: Stopping exploration...")
    os.killpg(ca_proc.pid, signal.SIGTERM)
    ca_proc.wait()
    ca_proc = None

    print("Step 8: Docking...")
    for i in range(3):
        try:
            tc.AutoDock(task_timeout=20.0)
        except TaskException:
            tc.Constant(linear=-0.1, angular=0.0, duration=7.5)

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
