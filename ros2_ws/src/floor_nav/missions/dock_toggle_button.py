#!/usr/bin/env python3
import math
import sys

import rclpy
from sensor_msgs.msg import Joy
from task_manager_client_py.TaskClient import TaskClient, TaskException


class DockToggleButton:
    def __init__(self, tc: TaskClient):
        self.tc = tc
        self._declare_parameters()

        self.joy_topic = self.tc.get_parameter('~/joy_topic').get_parameter_value().string_value
        self.toggle_button = self.tc.get_parameter('~/toggle_button').get_parameter_value().integer_value
        self.mux_service_name = self.tc.get_parameter('~/mux_service_name').get_parameter_value().string_value
        self.start_docked = self.tc.get_parameter('~/start_docked').get_parameter_value().bool_value
        self.reverse_duration = self.tc.get_parameter('~/undock_reverse_duration').get_parameter_value().double_value
        self.reverse_speed = self.tc.get_parameter('~/undock_reverse_speed').get_parameter_value().double_value
        self.turn_angle = self.tc.get_parameter('~/undock_turn_angle').get_parameter_value().double_value

        self.is_docked = self.start_docked
        self.last_pressed = False
        self.toggle_requested = False
        self.busy = False

        self.tc.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)
        self.tc.get_logger().info(
            'Dock toggle ready: button=%d topic=%s start_docked=%s'
            % (self.toggle_button, self.joy_topic, str(self.start_docked))
        )

    def _declare_parameters(self):
        self.tc.declare_parameter('~/joy_topic', '/joy')
        self.tc.declare_parameter('~/toggle_button', 2)
        self.tc.declare_parameter('~/mux_service_name', '/cmd_mux/select')
        self.tc.declare_parameter('~/start_docked', True)
        self.tc.declare_parameter('~/undock_reverse_duration', 1.20)
        self.tc.declare_parameter('~/undock_reverse_speed', 0.06)
        self.tc.declare_parameter('~/undock_turn_angle', math.pi)

    def joy_callback(self, msg: Joy):
        pressed = False
        if self.toggle_button < len(msg.buttons):
            pressed = msg.buttons[self.toggle_button] != 0

        if pressed and (not self.last_pressed) and (not self.busy):
            self.toggle_requested = True

        self.last_pressed = pressed

    def run_once(self):
        if (not self.toggle_requested) or self.busy:
            return

        self.toggle_requested = False
        self.busy = True
        try:
            if self.is_docked:
                self.tc.get_logger().info('Toggle pressed: undocking')
                self.undock()
                self.is_docked = False
            else:
                self.tc.get_logger().info('Toggle pressed: docking')
                self.dock()
                self.is_docked = True
        except TaskException as exc:
            self.tc.get_logger().error('Dock toggle task failed: %s' % str(exc))
        except Exception as exc:
            self.tc.get_logger().error('Dock toggle error: %s' % str(exc))
        finally:
            self.busy = False

    def undock(self):
        self.tc.SetMuxGeneric(topic='/mux/autoCommand', service_name=self.mux_service_name)
        self.tc.Constant(duration=self.reverse_duration, linear=-abs(self.reverse_speed), angular=0.0)
        self.tc.SetHeading(target=self.turn_angle, relative=True, max_angular_velocity=0.6, angle_threshold=0.05)

    def dock(self):
        self.tc.SetMuxGeneric(topic='/mux/autoCommand', service_name=self.mux_service_name)
        self.tc.AutoDock()


def main(argv=None):
    if argv is None:
        argv = sys.argv

    rclpy.init(args=argv)
    tc = TaskClient('/floor_tasks', 0.2)
    controller = DockToggleButton(tc)

    try:
        while rclpy.ok():
            rclpy.spin_once(tc, timeout_sec=0.1)
            controller.run_once()
    except KeyboardInterrupt:
        pass
    finally:
        tc.get_logger().info('Dock toggle button node exiting')


if __name__ == '__main__':
    main()
