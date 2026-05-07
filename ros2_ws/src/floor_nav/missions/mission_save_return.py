#!/usr/bin/env python3
import sys
import math
import time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)

# Capture current pose from SLAM (same approach as explore_dock_loop.py)
print("Saving initial position...")
captured = []

def _cb(msg: PoseWithCovarianceStamped):
    if not captured:
        p = msg.pose.pose
        q = p.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        captured.append((p.position.x, p.position.y, math.atan2(siny, cosy)))

sub1 = tc.create_subscription(PoseWithCovarianceStamped, '/pose', _cb, 10)
sub2 = tc.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', _cb, 10)

deadline = time.time() + 10.0
while not captured and time.time() < deadline:
    rclpy.spin_once(tc, timeout_sec=0.1)

tc.destroy_subscription(sub1)
tc.destroy_subscription(sub2)

if not captured:
    tc.get_logger().error("Could not capture pose from SLAM (is SLAM running?) — aborting")
    sys.exit(1)

init_x, init_y, init_theta = captured[0]
tc.get_logger().info(
    'Initial position saved: x=%.2f  y=%.2f  theta=%.1f deg'
    % (init_x, init_y, math.degrees(init_theta))
)

try:
    print("Step 1: Drive forward 3 seconds...")
    tc.Constant(linear=0.2, angular=0.0, duration=3.0)

    print("Step 2: Turn for 3 seconds...")
    tc.Constant(linear=0.0, angular=0.5, duration=3.0)

    print("Step 3: Drive forward 3 seconds...")
    tc.Constant(linear=0.2, angular=0.0, duration=3.0)

    print("Step 4: Returning to initial position (x=%.2f, y=%.2f, theta=%.1f deg)..."
          % (init_x, init_y, math.degrees(init_theta)))
    tc.GoToPose(
        goal_x=init_x,
        goal_y=init_y,
        goal_theta=init_theta,
        relative=False,
        smart=True,
        max_velocity=0.2,
        max_angular_velocity=1.0,
        dist_threshold=0.1,
        angle_threshold=0.2,
        k_v=1.5,
        k_alpha=2.0,
        k_beta=-1.5,
    )

    print("Mission completed! Robot returned to initial position.")

except TaskException as e:
    print("Exception caught: " + str(e))
