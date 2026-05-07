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
import tf2_geometry_msgs.tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from aruco_opencv_msgs.msg import ArucoDetection
import tf2_kdl

from ar_mapping_base.mapping_kf import *


class RoverMapping(Node):
    def __init__(self,name):
        super().__init__("rover_mapping")
        self.name = name
        self.declare_parameter('~/rover_name', self.name)
        self.declare_parameter('~/target_frame', "world")
        self.declare_parameter('~/ar_precision', 0.50)
        self.name = self.get_parameter('~/rover_name').get_parameter_value().string_value
        self.target_frame = self.get_parameter('~/target_frame').get_parameter_value().string_value
        self.ar_precision = self.get_parameter('~/ar_precision').get_parameter_value().double_value
        self.get_logger().info("Starting rover mapping for rover '%s'" % self.name)
        self.last_cmd = self.get_clock().now().nanoseconds/1e9
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ar_sub = self.create_subscription(ArucoDetection,"/aruco_detections", self.ar_cb, 1)
        self.connected = False

        # Instantiate the right filter based on launch parameters
        self.mapper = MappingKF(self)

    def ar_cb(self, markers):
        self.get_logger().info("Received marker array with %d detections" % len(markers.markers))
        for m in markers.markers:
            try:
                res = self.tf_buffer.can_transform(self.target_frame,'%s_ground'% self.name, markers.header.stamp, rclpy.time.Duration(seconds=1.0),True)
                if not res[0]:
                    self.get_logger().info(res[1])
                    continue
                res = self.tf_buffer.can_transform('%s_ground'% self.name, markers.header.frame_id, markers.header.stamp, rclpy.time.Duration(seconds=1.0),True)
                if not res[0]:
                    self.get_logger().info(res[1])
                    continue

                t = self.tf_buffer.lookup_transform(self.target_frame,'%s_ground'%(self.name),markers.header.stamp)
                F = tf2_kdl.transform_to_kdl(t)
                X = vstack([F.p.x(),F.p.y(),F.M.GetRPY()[2]])
                m_pose = PointStamped()
                m_pose.header = markers.header
                m_pose.point = m.pose.position
                t = self.tf_buffer.lookup_transform('%s_ground'%(self.name),markers.header.frame_id,markers.header.stamp)
                m_pose = tf2_geometry_msgs.do_transform_point(m_pose,t)
                Z = vstack([m_pose.point.x,m_pose.point.y])
                self.mapper.update_ar(self.get_logger(),Z,X,m.marker_id,self.ar_precision)
            except e:
                self.logger.error(f'{e}')
                continue
        self.mapper.publish(self.target_frame,markers.header.stamp)

def main(args=None):
    rclpy.init(args=args)

    rover_mapping = RoverMapping('rover')

    rclpy.spin(rover_mapping)

    rover_mapping.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

