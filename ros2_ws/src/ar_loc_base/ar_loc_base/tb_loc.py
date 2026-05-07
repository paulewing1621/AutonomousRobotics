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
from geometry_msgs.msg import PointStamped,Transform,TransformStamped
from aruco_opencv_msgs.msg import ArucoDetection
import tf2_kdl
import numpy as np
import PyKDL

from ar_loc_base.rover_kf import *
from ar_loc_base.rover_pf import *
from ar_loc_base.rover_odo import *



class DeltaLoc(Node):
    def __init__(self,name):
        super().__init__("rover_localisation")
        self.name = name
        self.declare_parameter('~/base_frame', "base_link")
        self.declare_parameter('~/target_frame', "world")
        self.declare_parameter('~/odom_frame', "odom")
        self.declare_parameter('~/use_ar', False)
        self.declare_parameter('~/filter_name', "particle")
        self.declare_parameter('~/x_precision', 0.01) #[m]
        self.declare_parameter('~/y_precision', 0.001) #[m]
        self.declare_parameter('~/theta_precision', 0.05) #[rad]
        self.declare_parameter('~/ar_precision', 0.50) #[m]
        self.declare_parameter('~/initial_x', -5.0) #[m]
        self.declare_parameter('~/initial_y', 2.5) #[m]
        self.declare_parameter('~/initial_theta', -pi/4) #[m]
        self.use_ar = self.get_parameter('~/use_ar').get_parameter_value().bool_value
        self.target_frame = self.get_parameter('~/target_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('~/odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('~/base_frame').get_parameter_value().string_value
        self.filter_name = self.get_parameter('~/filter_name').get_parameter_value().string_value
        self.x_precision = self.get_parameter('~/x_precision').get_parameter_value().double_value
        self.y_precision = self.get_parameter('~/y_precision').get_parameter_value().double_value
        self.theta_precision = self.get_parameter('~/theta_precision').get_parameter_value().double_value
        self.ar_precision = self.get_parameter('~/ar_precision').get_parameter_value().double_value
        self.initial_x = self.get_parameter('~/initial_x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('~/initial_y').get_parameter_value().double_value
        self.initial_theta = self.get_parameter('~/initial_theta').get_parameter_value().double_value
        self.get_logger().info("Starting turtlebot localisation" )
        self.last_cmd = self.get_clock().now().nanoseconds/1e9
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.ready = False
        self.connected = False
        self.last_odom = None
        self.lastTO = None



        # Instantiate the right filter based on launch parameters
        self.filter = None
        initial_vec = [self.initial_x, self.initial_y, self.initial_theta]
        initial_unc = [0.01, 0.01, 0.01]
        self.odom_unc = [self.x_precision,self.y_precision,self.theta_precision]
        if self.filter_name == "odo":
            self.filter = DeltaOdo(self,initial_vec, [1.0,1.0,1.0])
        elif self.filter_name == "kalman":
            self.filter = DeltaKF(self,initial_vec, [1.0,1.0,1.0])
        elif self.filter_name == "particle":
            self.filter = DeltaPF(self,initial_vec, [1.0,1.0,1.0])
        else:
            self.get_logger().error("Invalid filter name")
            raise SystemError

        self.get_logger().info("Setting up subscribers/publishers")
        self.ar_sub = self.create_subscription(ArucoDetection,"/aruco_detections", self.ar_cb, 1)
        self.odom_sub = self.create_subscription(Odometry,"/odom", self.odo_cb, 1)
        self.pose_pub = self.create_publisher(PoseStamped,"~/pose",1)
        self.odom_pub = self.create_publisher(Odometry,"~/odom",1)
        # print "Initialising wheel data structure"
        self.get_logger().info("Turtlebot is ready")
        self.ready = True

    def odo_cb(self,odom):
        if self.last_odom is None:
            self.last_odom = odom
            return

        last_pose = self.last_odom.pose.pose
        pose = odom.pose.pose
        self.last_odom = odom   
        tO=TransformStamped()
        tO.header=odom.header
        tO.child_frame_id=odom.child_frame_id
        tO.transform.translation.x=pose.position.x
        tO.transform.translation.y=pose.position.y
        tO.transform.translation.z=pose.position.z
        tO.transform.rotation=pose.orientation
        self.lastTO=tO

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
        deltap = [abs(z)*y+5e-3 for y,z in zip(self.odom_unc,deltax)]
        deltaX = np.vstack(deltax)
        deltaP = np.diag(deltap)
        self.get_logger().info("DeltaX: " + str(deltax)+ " DeltaP: " + str(deltap))

        self.filter.predict_delta(self.get_logger(),deltaX,deltaP,True)
        self.publish(odom.header.stamp,True)


    def publish(self, stamp, broadcast=True):
        self.filter.publish(self.pose_pub,self.odom_pub,self.target_frame,stamp,self.base_frame)
        if not broadcast:
            return
        if self.lastTO is not None:
            tO = self.lastTO
        else:
            return
        FO = tf2_kdl.transform_to_kdl(tO)
        FL = self.filter.getFrame()
        Fcorrection = FL * FO.Inverse()
        Tcorrection=TransformStamped()
        Tcorrection.header.stamp = tO.header.stamp
        Tcorrection.header.frame_id = self.target_frame
        Tcorrection.child_frame_id = self.odom_frame
        Tcorrection.transform.translation.x=Fcorrection.p.x()
        Tcorrection.transform.translation.y=Fcorrection.p.y()
        Tcorrection.transform.translation.z=Fcorrection.p.z()
        (qx,qy,qz,qw) = Fcorrection.M.GetQuaternion()
        Tcorrection.transform.rotation.x=qx
        Tcorrection.transform.rotation.y=qy
        Tcorrection.transform.rotation.z=qz
        Tcorrection.transform.rotation.w=qw
        self.broadcaster.sendTransform(Tcorrection)

    def ar_cb(self, markers):
        if not self.use_ar:
            return
        self.get_logger().info("Received marker array with %d detections" % len(markers.markers))
        for m in markers.markers:
            try:
                res = self.tf_buffer.can_transform(self.target_frame,'MARKER %02d A'% m.marker_id, rclpy.time.Time(), rclpy.time.Duration(seconds=1.0),True)
                if not res[0]:
                    self.get_logger().info(res[1])
                    continue
                res = self.tf_buffer.can_transform(self.target_frame,'MARKER %02d B'% m.marker_id, rclpy.time.Time(), rclpy.time.Duration(seconds=1.0),True)
                if not res[0]:
                    self.get_logger().info(res[1])
                    continue
                res = self.tf_buffer.can_transform(self.base_frame, markers.header.frame_id, markers.header.stamp, rclpy.time.Duration(seconds=1.0),True)
                if not res[0]:
                    self.get_logger().info(res[1])
                    continue

                tA = self.tf_buffer.lookup_transform(self.target_frame,'MARKER %02d A'%m.marker_id,rclpy.time.Time())
                FA = tf2_kdl.transform_to_kdl(tA)
                tB = self.tf_buffer.lookup_transform(self.target_frame,'MARKER %02d B'%m.marker_id,rclpy.time.Time())
                FB = tf2_kdl.transform_to_kdl(tB)
                L = vstack([(FA.p.x()+FB.p.x())/2,(FA.p.y()+FB.p.y())/2])
                m_pose = PointStamped()
                m_pose.header = markers.header
                m_pose.point = m.pose.position
                t = self.tf_buffer.lookup_transform(self.base_frame,markers.header.frame_id,markers.header.stamp)
                m_pose = tf2_geometry_msgs.do_transform_point(m_pose,t)
                Z = vstack([m_pose.point.x,m_pose.point.y])

                # TODO
                self.filter.update_ar(self.get_logger(),Z,L,self.ar_precision)
            except e:
                self.logger.error(f'{e}')
                continue
        self.publish(markers.header.stamp,True)

def main(args=None):
    rclpy.init(args=args)

    delta_loc = DeltaLoc('turtlebot')

    rclpy.spin(delta_loc)

    rover_loc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

