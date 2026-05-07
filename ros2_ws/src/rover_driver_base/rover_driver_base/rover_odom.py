#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist,Pose
from nav_msgs.msg import Odometry
from math import atan2, hypot, pi, cos, sin
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion, TransformStamped
import message_filters
import numpy as np
import math
from numpy.linalg import pinv

from rover_driver_base.rover_kinematics import *

class RoverDriver(Node):
    def __init__(self,name):
        super().__init__("rover_odom")
        self.name = name
        self.declare_parameter('~/rover_name', self.name)
        self.declare_parameter('~/publish_tf', True)
        self.declare_parameter('~/publish_odom', True)
        self.name = self.get_parameter('~/rover_name').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('~/publish_tf').get_parameter_value().bool_value
        self.publish_odom = self.get_parameter('~/publish_odom').get_parameter_value().bool_value
        self.get_logger().info("Starting rover odom for rover '%s'" % self.name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry,"~/odom",1)
        if self.publish_tf:
            self.broadcaster = TransformBroadcaster(self)
        self.drive_sub={}
        self.steering_sub={}
        self.drive_pub={}
        self.ready = False
        self.connected = False

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


        self.get_logger().info("Setting up subscribers/publishers")
        # print "Initialising wheel data structure"
        for k in prefix:
            self.steering_sub[k] = message_filters.Subscriber(self, JointState,"/vrep/%s/%sSteerEncoder" % (self.name,k))
            self.drive_sub[k] = message_filters.Subscriber(self, JointState,"/vrep/%s/%sDriveEncoder" % (self.name,k))
            # print "Initialised wheel " + k
        self.ts = message_filters.TimeSynchronizer(list(self.steering_sub.values())+list(self.drive_sub.values()), 50)
        self.ts.registerCallback(self.sync_odo_cb)
        self.get_logger().info("Rover is ready")
        self.ready = True

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
            except TransformExpection as e:
                self.get_logger().warn(f"{e}")
            now = self.get_clock().now().nanoseconds/1e9
            if now - t0 > duration:
                return False
            #rate.sleep()



    def sync_odo_cb(self,*args):
        self.connected = True
        if not self.ready:
            return
        if len(args)!=12:
            rospy.logerr("Invalid number of argument in OdoCallback")
            return
        steering_val = [s.position[0] for s in args[0:6]]
        drive_val = [s.position[0] for s in args[6:12]]
        motors = RoverMotors()
        motors.steering = dict(zip(self.steering_sub.keys(),steering_val))
        motors.drive = dict(zip(self.drive_sub.keys(),drive_val))
        self.odo_cb(args[0].header.stamp,motors)

    def odo_cb(self,timestamp,motors):
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
        X = self.kinematics.integrate_odometry(motors, drive_cfg)

        if self.publish_odom:
            # Publishing the inverse transform to avoid tree problems in TF
            t = Odometry()

            # Read message content and assign it to
            # corresponding tf variables
            t.header.stamp = timestamp
            t.header.frame_id = "odom"
            t.child_frame_id = to_frame_rel

            # coordinates from the message and set the z coordinate to 0
            t.pose.pose.position.x = X[0,0]
            t.pose.pose.position.y = X[1,0]
            t.pose.pose.position.z = 0.0

            # For the same reason, turtle can only rotate around one axis
            # and this why we set rotation in x and y to 0 and obtain
            # rotation in z axis from the message
            q = quaternion_from_euler(0, 0, X[2,0])
            t.pose.pose.orientation.x = q[0]
            t.pose.pose.orientation.y = q[1]
            t.pose.pose.orientation.z = q[2]
            t.pose.pose.orientation.w = q[3]

            # Send the transformation
            self.odom_pub.publish(t)

        if self.publish_tf:
            # Publishing the inverse transform to avoid tree problems in TF
            t = TransformStamped()

            # Read message content and assign it to
            # corresponding tf variables
            t.header.stamp = timestamp
            t.header.frame_id = to_frame_rel
            t.child_frame_id = "odom"

            # coordinates from the message and set the z coordinate to 0
            t.transform.translation.x = -(cos(X[2,0])*X[0,0] + sin(X[2,0])*X[1,0])
            t.transform.translation.y = -(-sin(X[2,0])*X[0,0]+cos(X[2,0])*X[1,0])
            t.transform.translation.z = 0.0

            # For the same reason, turtle can only rotate around one axis
            # and this why we set rotation in x and y to 0 and obtain
            # rotation in z axis from the message
            q = quaternion_from_euler(0, 0, -X[2,0])
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            # Send the transformation
            self.broadcaster.sendTransform(t)



def main(args=None):
    rclpy.init(args=args)

    rover_odom = RoverDriver('rover')

    rclpy.spin(rover_odom)

    rover_odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
