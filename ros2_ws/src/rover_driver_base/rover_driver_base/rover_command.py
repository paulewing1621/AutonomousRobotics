#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time,Duration
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist,Pose
from math import atan2, hypot, pi, cos, sin
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import message_filters
import numpy
from numpy.linalg import pinv

from rover_driver_base.rover_kinematics import *

def float64(x):
    y = Float64()
    y.data = float(x)
    return y

class RoverDriver(Node):
    def __init__(self,name):
        super().__init__("rover_command")
        self.name = name
        self.declare_parameter('~/rover_name', self.name)
        self.declare_parameter('~/skidsteer', False)
        self.declare_parameter('~/check_timeout', True)
        self.name = self.get_parameter('~/rover_name').get_parameter_value().string_value
        self.skidsteer = self.get_parameter('~/skidsteer').get_parameter_value().bool_value
        self.check_timeout = self.get_parameter('~/check_timeout').get_parameter_value().bool_value
        self.get_logger().info("Starting rover command for rover '%s'" % self.name)
        self.last_cmd = self.get_clock().now().nanoseconds/1e9
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.drive_pub={}
        self.drive_sub={}
        self.steering_pub={}
        self.steering_sub={}
        self.drive_pub={}
        self.ready = False

        self.drive_state = None
        self.kinematics = RoverKinematics()

        self.get_logger().info("Waiting for initial transforms")
        self.radius={}
        for k in prefix:
            try:
                if not self.waittf('%s_ground'%(self.name), '%sDrive'%k, 10.0):
                    raise TransformException
                t = self.tf_buffer.lookup_transform('%s_ground'%(self.name),
                        '%sDrive'%k, rclpy.time.Time())
                self.radius[k] = t.transform.translation.z
                self.get_logger().info("Got transform for " + k)
            except TransformException as ex:
                self.get_logger().error("TF exception: " + repr(ex))

        self.ready = True
        self.get_logger().info("Setting up subscribers/publishers")

        self.twist_sub = self.create_subscription(Twist, '~/twistCommand', self.twist_cb, 1)
        # print "Initialising wheel data structure"
        for k in prefix:
            self.steering_pub[k] = self.create_publisher(Float64,"/vrep/%s/%sSteerCommand" % (self.name,k), 1)
            self.drive_pub[k] = self.create_publisher(Float64,"/vrep/%s/%sDriveCommand" % (self.name,k), 1)
        # print "Initialising wheel data structure"
        for k in prefix:
            self.steering_sub[k] = message_filters.Subscriber(self, JointState,"/vrep/%s/%sSteerEncoder" % (self.name,k))
            self.drive_sub[k] = message_filters.Subscriber(self, JointState,"/vrep/%s/%sDriveEncoder" % (self.name,k))
            # print "Initialised wheel " + k
        self.ts = message_filters.TimeSynchronizer(list(self.steering_sub.values())+list(self.drive_sub.values()), 50)
        self.ts.registerCallback(self.sync_odo_cb)

        self.timeout = True
        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info("Rover is ready")

    def waittf(self,tf_from,tf_to,duration):
        rate = self.create_rate(2)
        t0 = self.get_clock().now().nanoseconds/1e9
        while rclpy.ok():
            self.get_logger().info("waiting...")
            rclpy.spin_once(self,timeout_sec=0.5)
            try:
                res = self.tf_buffer.can_transform(tf_from,tf_to,rclpy.time.Time(),rclpy.time.Duration(seconds=1.),True)
                if res[0]:
                    return True
                self.get_logger().warn(res[1])
            except TransformException as e:
                self.get_logger().warn(f"{e}")
            now = self.get_clock().now().nanoseconds/1e9
            if now - t0 > duration:
                return False
            # rate.sleep()



    def sync_odo_cb(self,*args):
        self.connected = True
        if not self.ready:
            return
        if len(args)!=12:
            rospy.logerr("Invalid number of argument in OdoCallback")
            return
        steering_val = [s.position[0] for s in args[0:6]]
        drive_val = [s.position[0] for s in args[6:12]]
        self.drive_state = RoverMotors()
        self.drive_state.steering = dict(zip(self.steering_sub.keys(),steering_val))
        self.drive_state.drive = dict(zip(self.drive_sub.keys(),drive_val))



    def twist_cb(self,twist):
        if not self.ready:
            return
        self.last_cmd = self.get_clock().now().nanoseconds/1e9
        # Get the pose of all drives
        drive_cfg={}
        to_frame_rel = '%s_ground'%(self.name)
        for k in prefix:
            try:
                from_frame_rel = '%sDrive'%k
                t = self.tf_buffer.lookup_transform(
                    to_frame_rel, from_frame_rel, rclpy.time.Time())
                t = t.transform.translation
                drive_cfg[k] = DriveConfiguration(self.radius[k],t.x,t.y,t.z)
            except TransformException as ex:
                self.get_logger().info( f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                return

        # Now compute for each drive, its rotation speed and steering angle
        motors = self.kinematics.twist_to_motors(twist,drive_cfg,self.skidsteer,self.drive_state)
        self.publish(motors)

    def publish(self, motor):
        for k in prefix:
            self.drive_pub[k].publish(float64(motor.drive[k]))
            self.steering_pub[k].publish(float64(motor.steering[k]))
            

    def on_timer(self):
        now = self.get_clock().now().nanoseconds/1e9
        if self.check_timeout:
            if (now - self.last_cmd) < 0.5: 
                if self.timeout:
                    self.timeout = False
                    self.get_logger().info("Accepting joystick commands")
            else:
                if not self.timeout:
                    self.timeout = True
                    self.get_logger().info("Timeout: ignoring joystick commands")
                motors = RoverMotors()
                self.publish(motors)


def main(args=None):
    rclpy.init(args=args)

    rover_command = RoverDriver('rover')

    rclpy.spin(rover_command)

    rover_command.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
