#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos, atan2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import threading
from rover_driver_base.rover_kinematics import RoverKinematics
from ar_loc_base.rover_odo import RoverOdo, DeltaOdo

class DeltaKF(DeltaOdo):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node,initial_pose, initial_uncertainty)
        self.X = mat(vstack(initial_pose))
        self.P = mat(diag(initial_uncertainty))
        self.ellipse_pub = node.create_publisher(Marker,"~/ellipse",1)
        self.pose_with_cov_pub = node.create_publisher(PoseWithCovarianceStamped,"~/pose_with_covariance",1)

    def getRotationFromWorldToRobot(self):
        return self.getRotation(-self.X[2,0])

    def predict_delta(self, logger, DeltaX, CovDeltaX, lock=True):
        if lock:
            self.lock.acquire()
        
        theta = self.X[2,0]
        dx = DeltaX[0,0]
        dy = DeltaX[1,0]

        self.X = self.X + self.getRotationFromWorldToRobot() @ DeltaX

        F = np.eye(3)
        F[0,2] = -np.sin(theta) * dx - np.cos(theta) * dy
        F[1,2] =  np.cos(theta) * dx - np.sin(theta) * dy


        G = self.getRotationFromWorldToRobot(theta)

        self.P = F @ self.F @ F.T + G @ CovDeltaX @ G.T

        self.lock.release()
        return (self.X,self.P)

    def update_ar(self, logger, Z, L, uncertainty):
        self.lock.acquire()

        logger.info("Update: L="+str(L.T)+" X="+str(self.X.T))

        theta = self.X[2,0]
        dx = L[0,0]-self.X[0,0]
        dy = L[1,0]-self.X[1,0]
        
        Z_p = np.mat([
            [ dx * cos(theta) + dy * sin(theta)],
            [-dx * sin(theta) + dy * cos(theta)]
        ])
        
        H = np.mat([
            [-cos(theta), -sin(theta), -dx*sin(theta) + dy*cos(theta)],
            [ sin(theta), -cos(theta), -dx*cos(theta) - dy*sin(theta)]
        ])
        
        Y = Z - Z_p
        R = np.mat(diag(uncertainty)) 
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ inv(S)
        
        self.X = self.X + K @ Y
        self.P = (np.eye(3) - K @ H) @ self.P
        
        self.lock.release()
        return (self.X, self.P)

    def update_compass(self, logger, Z, uncertainty):
        self.lock.acquire()

        logger.info("Update: S="+str(Z)+" X="+str(self.X.T))

        tetha_p = self.X[2,0]
        Y = Z - tetha_p
        Y = (Y+np.pi) % (2*np.pi) - np.pi 

        H = np.mat([[0, 0, 1]])

        R = np.mat([[uncertainty]]) 
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.X = self.X + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

        self.lock.release()
        return (self.X,self.P)

    def publish(self, pose_pub, odom_pub, target_frame, stamp, child_frame):
        pose_simple = super().publish(pose_pub, odom_pub, target_frame, stamp, child_frame)
        pose = PoseWithCovarianceStamped()
        pose.header = pose_simple.header
        pose.pose.pose = pose_simple.pose
        C = [0.]*36
        C[ 0] = self.P[0,0]; C[ 1] = self.P[0,1]; C[ 5] = self.P[0,2]
        C[ 6] = self.P[1,0]; C[ 7] = self.P[1,1]; C[11] = self.P[1,2]
        C[30] = self.P[2,0]; C[31] = self.P[2,1]; C[35] = self.P[2,2]
        pose.pose.covariance = C
        self.pose_with_cov_pub.publish(pose)

        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose.pose
        marker.scale.x = 3*sqrt(self.P[0,0])
        marker.scale.y = 3*sqrt(self.P[1,1])
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.ellipse_pub.publish(marker)


class RoverKF(DeltaKF):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node,initial_pose,initial_uncertainty)
        self.kinematics=RoverKinematics()

    def predict(self, logger, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        if self.first_run:
            self.kinematics.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return (self.X, self.P)
        iW = self.kinematics.prepare_inversion_matrix(drive_cfg)
        S = self.kinematics.prepare_displacement_matrix(self.kinematics.motor_state, motor_state, drive_cfg)
        self.kinematics.motor_state.copy(motor_state)
        DeltaX = iW @ S
        num_motors = len(S)
        CovS = np.eye(num_motors) * (encoder_precision**2)
        CovDeltaX = iW @ CovS @ iW.T
        self.predict_delta(logger, DeltaX, CovDeltaX, False)
        self.lock.release()
        return (self.X, self.P)

        

