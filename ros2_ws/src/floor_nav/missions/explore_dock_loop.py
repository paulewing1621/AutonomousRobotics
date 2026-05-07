#!/usr/bin/env python3
"""
explore_dock_loop.py  —  Autonomous explore-then-dock mission
=============================================================
Place this file in: floor_nav/missions/

Sequence
--------
1. Read home pose from parameters (robot starts docked → current pose = home)
2. Disable collision avoidance (CA) so the undock reverse works cleanly
3. Undock  : reverse blindly → turn 180°
4. Re-enable CA
5. Enable  exploration on the occgrid planner
6. Wait    for explore_seconds
7. Disable exploration
8. Navigate to a pre-dock staging pose (offset in front of dock)
9. Disable CA again before the final approach
10. AutoDock with up to dock_retries attempts; on each failure:
      reverse slightly → re-plan to staging pose → retry
11. Re-enable CA
12. Mission complete
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import SetBool
from task_manager_client_py.TaskClient import TaskClient, TaskException


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def call_setbool(node, service_name: str, value: bool, timeout_sec: float = 5.0) -> bool:
    """Call a std_srvs/SetBool service. Returns True on success, False on failure."""
    cli = node.create_client(SetBool, service_name)
    deadline = time.time() + timeout_sec
    while (not cli.wait_for_service(timeout_sec=0.25)) and (time.time() < deadline):
        pass

    if not cli.service_is_ready():
        node.get_logger().warning(
            'Service %s not available within %.1f s — skipping' % (service_name, timeout_sec)
        )
        return False

    req = SetBool.Request()
    req.data = value
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if (not future.done()) or (future.result() is None):
        node.get_logger().warning('Call to %s timed out' % service_name)
        return False

    res = future.result()
    if not res.success:
        node.get_logger().warning('%s returned failure: %s' % (service_name, res.message))
        return False

    node.get_logger().info('%s → %s (%s)' % (service_name, str(value), res.message))
    return True


# ---------------------------------------------------------------------------
# Mission class
# ---------------------------------------------------------------------------

class ExploreDockMission:

    def __init__(self, tc: TaskClient):
        self.tc = tc
        self._declare_parameters()

        gp = self.tc.get_parameter

        self.home_x              = gp('~/home_x').get_parameter_value().double_value
        self.home_y              = gp('~/home_y').get_parameter_value().double_value
        self.home_theta          = gp('~/home_theta').get_parameter_value().double_value
        self.auto_capture_home   = gp('~/auto_capture_home').get_parameter_value().bool_value
        self.reverse_duration    = gp('~/undock_reverse_duration').get_parameter_value().double_value
        self.reverse_speed       = gp('~/undock_reverse_speed').get_parameter_value().double_value
        self.turn_angle          = gp('~/undock_turn_angle').get_parameter_value().double_value
        self.mux_service_name    = gp('~/mux_service_name').get_parameter_value().string_value
        self.ca_enable_service   = gp('~/ca_enable_service').get_parameter_value().string_value
        self.predock_offset      = gp('~/predock_offset').get_parameter_value().double_value
        self.explore_seconds     = gp('~/explore_seconds').get_parameter_value().double_value
        self.dock_retries        = gp('~/dock_retries').get_parameter_value().integer_value
        self.dock_retry_reverse  = gp('~/dock_retry_reverse').get_parameter_value().double_value
        self.dock_retry_reverse_speed = gp('~/dock_retry_reverse_speed').get_parameter_value().double_value
        self.predock_dist_threshold   = gp('~/predock_dist_threshold').get_parameter_value().double_value

        if self.auto_capture_home:
            self._try_capture_home_pose()

    # ------------------------------------------------------------------
    # Parameter declarations
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        self.tc.declare_parameter('~/home_x', 0.0)
        self.tc.declare_parameter('~/home_y', 0.0)
        self.tc.declare_parameter('~/home_theta', 0.0)
        self.tc.declare_parameter('~/auto_capture_home', True)
        self.tc.declare_parameter('~/predock_offset', 0.40)
        self.tc.declare_parameter('~/predock_dist_threshold', 0.20)
        self.tc.declare_parameter('~/explore_seconds', 120.0)
        self.tc.declare_parameter('~/dock_retries', 3)
        self.tc.declare_parameter('~/mux_service_name', '/cmd_mux/select')
        self.tc.declare_parameter('~/ca_enable_service', '/collision_avoidance/enable')
        self.tc.declare_parameter('~/undock_reverse_duration', 1.20)
        self.tc.declare_parameter('~/undock_reverse_speed', 0.06)
        self.tc.declare_parameter('~/undock_turn_angle', math.pi)
        self.tc.declare_parameter('~/dock_retry_reverse', 0.80)
        self.tc.declare_parameter('~/dock_retry_reverse_speed', 0.05)

    # ------------------------------------------------------------------
    # Auto-capture home pose from SLAM at start-up
    # ------------------------------------------------------------------

    def _try_capture_home_pose(self):
        """
        Listen on /pose (SLAM Toolbox) or /amcl_pose for up to 3 seconds.
        Stores the result as the home pose (robot must be docked at this point).
        Falls back to parameter values if no message arrives.
        """
        captured = []

        def _cb(msg: PoseWithCovarianceStamped):
            if not captured:
                p = msg.pose.pose
                yaw = self._quat_to_yaw(p.orientation)
                captured.append((p.position.x, p.position.y, yaw))

        sub1 = self.tc.create_subscription(PoseWithCovarianceStamped, '/pose', _cb, 10)
        sub2 = self.tc.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', _cb, 10)

        deadline = time.time() + 3.0
        while (not captured) and (time.time() < deadline):
            rclpy.spin_once(self.tc, timeout_sec=0.1)

        self.tc.destroy_subscription(sub1)
        self.tc.destroy_subscription(sub2)

        if captured:
            self.home_x, self.home_y, self.home_theta = captured[0]
            self.tc.get_logger().info(
                'Home pose captured from SLAM: x=%.3f  y=%.3f  θ=%.3f rad'
                % (self.home_x, self.home_y, self.home_theta)
            )
        else:
            self.tc.get_logger().warning(
                'Could not capture home pose from SLAM — '
                'using parameter values (x=%.3f y=%.3f θ=%.3f)'
                % (self.home_x, self.home_y, self.home_theta)
            )

    @staticmethod
    def _quat_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ------------------------------------------------------------------
    # CA enable / disable
    # ------------------------------------------------------------------

    def _ca_enable(self, enabled: bool):
        call_setbool(self.tc, self.ca_enable_service, enabled, timeout_sec=5.0)

    # ------------------------------------------------------------------
    # Exploration toggle
    # ------------------------------------------------------------------

    def set_exploration(self, enabled: bool):
        ok = call_setbool(self.tc, '/occgrid_planner/set_exploration', enabled, timeout_sec=5.0)
        if not ok:
            raise RuntimeError(
                'Failed to %s exploration' % ('enable' if enabled else 'disable')
            )

    # ------------------------------------------------------------------
    # Undock
    # ------------------------------------------------------------------

    def undock(self):
        self.tc.get_logger().info('Undocking: disabling CA → reverse → turn 180°')
        self._ca_enable(False)
        try:
            self.tc.SetMuxGeneric(topic='/mux/autoCommand', service_name=self.mux_service_name)
            self.tc.Constant(
                duration=self.reverse_duration,
                linear=-abs(self.reverse_speed),
                angular=0.0
            )
            self.tc.SetHeading(
                target=self.turn_angle,
                relative=True,
                max_angular_velocity=0.6,
                angle_threshold=0.05
            )
        finally:
            self._ca_enable(True)
            self.tc.get_logger().info('CA re-enabled after undock')

    # ------------------------------------------------------------------
    # Pre-dock staging pose
    # ------------------------------------------------------------------

    def _predock_pose(self):
        pre_x = self.home_x + self.predock_offset * math.cos(self.home_theta)
        pre_y = self.home_y + self.predock_offset * math.sin(self.home_theta)
        return pre_x, pre_y, self.home_theta

    # ------------------------------------------------------------------
    # Return to base and dock
    # ------------------------------------------------------------------

    def return_to_base_and_dock(self) -> bool:
        pre_x, pre_y, home_theta = self._predock_pose()

        self.tc.get_logger().info(
            'Navigating to pre-dock staging pose (%.3f, %.3f, %.3f rad)'
            % (pre_x, pre_y, home_theta)
        )
        self.tc.PlanTo(
            goal_x=pre_x, goal_y=pre_y, goal_theta=home_theta,
            dist_threshold=self.predock_dist_threshold,
            pub_period=1.0
        )

        self._ca_enable(False)
        try:
            for attempt in range(self.dock_retries):
                try:
                    self.tc.get_logger().info(
                        'AutoDock attempt %d / %d' % (attempt + 1, self.dock_retries)
                    )
                    self.tc.AutoDock()
                    self.tc.get_logger().info('Docking succeeded ✓')
                    return True

                except TaskException as exc:
                    self.tc.get_logger().warning(
                        'AutoDock attempt %d failed: %s' % (attempt + 1, str(exc))
                    )
                    if attempt + 1 < self.dock_retries:
                        self.tc.get_logger().info('Recovery: reversing then replanning to staging pose')
                        self.tc.Constant(
                            duration=self.dock_retry_reverse,
                            linear=-abs(self.dock_retry_reverse_speed),
                            angular=0.0
                        )
                        # Re-enable CA for the replanning drive, then disable again
                        self._ca_enable(True)
                        self.tc.PlanTo(
                            goal_x=pre_x, goal_y=pre_y, goal_theta=home_theta,
                            dist_threshold=self.predock_dist_threshold,
                            pub_period=1.0
                        )
                        self._ca_enable(False)
        finally:
            self._ca_enable(True)
            self.tc.get_logger().info('CA re-enabled after docking sequence')

        return False

    # ------------------------------------------------------------------
    # Top-level run
    # ------------------------------------------------------------------

    def run(self):
        self.undock()

        self.set_exploration(True)
        self.tc.get_logger().info(
            'Exploration started — running for %.1f seconds' % self.explore_seconds
        )
        self.tc.Wait(duration=self.explore_seconds)

        self.set_exploration(False)
        self.tc.get_logger().info('Exploration finished — returning to base')

        success = self.return_to_base_and_dock()
        if success:
            self.tc.get_logger().info('Mission complete: robot is docked ✓')
        else:
            self.tc.get_logger().error(
                'Docking failed after %d attempts — robot stopped in auto mode'
                % self.dock_retries
            )
            self.tc.Constant(duration=0.5, linear=0.0, angular=0.0)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(argv=None):
    if argv is None:
        argv = sys.argv

    rclpy.init(args=argv)
    tc = TaskClient('/floor_tasks', 0.2)
    mission = ExploreDockMission(tc)

    try:
        mission.run()
    except (TaskException, RuntimeError) as exc:
        tc.get_logger().error('Mission aborted: %s' % str(exc))
        try:
            mission.set_exploration(False)
        except Exception:
            pass
        try:
            mission._ca_enable(True)
        except Exception:
            pass
    finally:
        tc.get_logger().info('Mission node exiting')


if __name__ == '__main__':
    main()
