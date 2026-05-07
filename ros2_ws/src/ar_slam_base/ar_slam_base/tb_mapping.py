#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.time import Time,Duration
from std_msgs.msg import Float64,Float32
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Pose
from math import atan2, hypot, pi, cos, sin
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs.tf2_geometry_msgs
import message_filters
from geometry_msgs.msg import PointStamped
from aruco_opencv_msgs.msg import ArucoDetection
import tf2_kdl
import numpy as np

from ar_slam_base.mapping_kf import *
import numpy as np
import PyKDL



class DeltaMapping(Node):
    def __init__(self,name):
        super().__init__("rover_mapping")
        self.name = name
        self.declare_parameter('~/target_frame', "map")
        self.declare_parameter('~/base_frame', "base_link")
        self.declare_parameter('~/ignore_id', False)
        self.declare_parameter('~/x_precision', 0.01) #[m]
        self.declare_parameter('~/y_precision', 0.001) #[m]
        self.declare_parameter('~/theta_precision', 0.05) #[rad]
        self.declare_parameter('~/ar_precision', 0.50) #[m]
        self.declare_parameter('~/initial_x', 0.0) #[m]
        self.declare_parameter('~/initial_y', 0.0) #[m]
        self.declare_parameter('~/initial_theta', 0.0) #[rad]
        self.ignore_id = self.get_parameter('~/ignore_id').get_parameter_value().bool_value
        self.target_frame = self.get_parameter('~/target_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('~/base_frame').get_parameter_value().string_value
        self.x_precision = self.get_parameter('~/x_precision').get_parameter_value().double_value
        self.y_precision = self.get_parameter('~/y_precision').get_parameter_value().double_value
        self.theta_precision = self.get_parameter('~/theta_precision').get_parameter_value().double_value
        self.ar_precision = self.get_parameter('~/ar_precision').get_parameter_value().double_value
        self.initial_x = self.get_parameter('~/initial_x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('~/initial_y').get_parameter_value().double_value
        self.initial_theta = self.get_parameter('~/initial_theta').get_parameter_value().double_value
        self.get_logger().info("Starting turtlebot SLAM")
        self.last_cmd = self.get_clock().now().nanoseconds/1e9
        self.last_odom = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.ready = False
        self.connected = False

        # Instantiate the right filter based on launch parameters
        initial_vec = [self.initial_x, self.initial_y, self.initial_theta]
        initial_unc = [0.01, 0.01, 0.01]
        self.odom_unc = [self.x_precision,self.y_precision,self.theta_precision]
        self.mapper = MappingKF(self,initial_vec,initial_unc)


        self.get_logger().info("Setting up subscribers/publishers")
        self.ar_sub = self.create_subscription(ArucoDetection,"/aruco_detections", self.ar_cb, 1)
        self.odom_sub = self.create_subscription(Odometry,"/odom", self.odo_cb, 1)
        self.pose_pub = self.create_publisher(PoseStamped,"~/pose",1)
        self.odom_pub = self.create_publisher(Odometry,"~/odom",1)
        # print "Initialising wheel data structure"
        self.get_logger().info("Turtlebot is ready")
        self.ready = True

    def publish(self, stamp):
        self.mapper.publish(self.pose_pub,self.odom_pub,self.target_frame,stamp,self.base_frame)
        self.mapper.broadcast(self.broadcaster, self.target_frame, self.base_frame, stamp, True)


    def odo_cb(self,odom):
        if self.last_odom is None:
            self.last_odom = odom
            return

        last_pose = self.last_odom.pose.pose
        pose = odom.pose.pose
        self.last_odom = odom

        pp = last_pose.position
        pq = last_pose.orientation
        cp = pose.position
        cq = pose.orientation
        prev_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(pq.x, pq.y, pq.z, pq.w),
                PyKDL.Vector(pp.x,pp.y,pp.z))
        cur_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(cq.x, cq.y, cq.z, cq.w),
                PyKDL.Vector(cp.x,cp.y,cp.z))
        delta_frame = prev_frame.Inverse()*cur_frame
        
        deltax = [delta_frame.p.x(),delta_frame.p.y(),delta_frame.M.GetRPY()[2]]
        deltap = [abs(z)*y+1e-12 for y,z in zip(self.odom_unc,deltax)]
        deltaX = np.vstack(deltax)
        deltaP = np.diag(deltap)
        self.get_logger().info("DeltaX: " + str(deltax)+ " DeltaP: " + str(deltap))

        self.mapper.predict_delta(self.get_logger(),deltaX,deltaP)
        self.publish(odom.header.stamp)



    def ar_cb(self, markers):
        self.get_logger().info("Received marker array with %d detections" % len(markers.markers))
        for m in markers.markers:
            try:
                res = self.tf_buffer.can_transform(self.base_frame, markers.header.frame_id, markers.header.stamp, rclpy.time.Duration(seconds=1.0),True)
                if not res[0]:
                    self.get_logger().info(res[1])
                    continue
                m_pose = PointStamped()
                m_pose.header = markers.header
                m_pose.point = m.pose.position
                t = self.tf_buffer.lookup_transform(self.base_frame,markers.header.frame_id,markers.header.stamp)
                m_pose = tf2_geometry_msgs.do_transform_point(m_pose,t)
                Z = np.vstack([m_pose.point.x,m_pose.point.y])

                if self.ignore_id:
                    self.mapper.update_ar(self.get_logger(),Z,0,self.ar_precision)
                else:
                    self.mapper.update_ar(self.get_logger(),Z,m.marker_id,self.ar_precision)
            except BaseException as e:
                self.get_logger().error(f'{e}')
                continue
        self.publish(markers.header.stamp)



def main(args=None):
    rclpy.init(args=args)

    delta_mapping = DeltaMapping('turtlebot')

    rclpy.spin(delta_mapping)

    delta_mapping.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

