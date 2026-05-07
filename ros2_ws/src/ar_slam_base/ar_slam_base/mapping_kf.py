#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from numpy.linalg import inv
from math import pi, sin, cos,hypot,sqrt
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import threading
from rover_driver_base.rover_kinematics import RoverKinematics
from ar_loc_base.rover_odo import RoverOdo
from rover_driver_base.rover_kinematics import RoverMotors 


class MappingKF(RoverOdo):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node,initial_pose,initial_uncertainty)
        self.lock = threading.Lock()
        self.X = np.mat(np.vstack(initial_pose))
        self.P = np.mat(np.diag(initial_uncertainty))
        self.idx = {}
        self.motor_state = RoverMotors()
        self.marker_pub = node.create_publisher(MarkerArray,"~/landmarks",1)
        self.kinematics = RoverKinematics()

    def predict_rover(self, logger, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return (self.X, self.P)
        # print("-"*32)
        # then compute odometry using least square
        iW = self.kinematics.prepare_inversion_matrix(drive_cfg)
        S = self.kinematics.prepare_displacement_matrix(self.motor_state, motor_state, drive_cfg)
        self.motor_state.copy(motor_state)
        
        # Implement Kalman prediction here
        # Compute the update in the body frame and the resulting uncertainty in the body frame
        DeltaX = iW @ S
        DeltaP = np.mat(np.diag([encoder_precision,encoder_precision,encoder_precision*0.1]))
        DeltaP = np.zeros((3,3))
        self.lock.release()
        return self.predict_delta(logger,DeltaX,DeltaP)

    def predict_delta(self, logger, DeltaX, DeltaP):
        self.lock.acquire()
        # Update the state using the provided displacement, but we only need to deal with a subset of the state
        # Assumption: deltaX and deltaP are defined in the body frame and need to be rotated to account for the jacobian 
        # of the transfer function
        # TODO
        # theta = self.X[2,0]
        # Rtheta = np.mat([[cos(theta), -sin(theta), 0], 
        #               [sin(theta),  cos(theta), 0],
        #               [         0,           0, 1]])

        # self.X[0:3,0] = self.X[0:3,0] + Rtheta @ DeltaX

        # Gx = np.eye(self.X.shape[0])
        # dx, dy = DeltaX[0, 0], DeltaX[1,0]
        # Gx[0,2] = -sin(theta)*dx - cos(theta)*dy
        # Gx[1,2] =  cos(theta)*dx - sin(theta)*dy

        # Q = np.zeros((self.X.shape[0], self.X.shape[0]))
        # Q[0:3,0:3] = Rtheta @ DeltaP @ Rtheta.T

        # self.P = Gx @ self.P @ Gx.T + Q

        theta = float(self.X[2, 0])
        cos_t, sin_t = cos(theta), sin(theta)
        dx, dy = float(DeltaX[0, 0]), float(DeltaX[1, 0])
        R_theta = np.mat([[cos_t, -sin_t, 0],
                    [sin_t,  cos_t, 0],
                    [    0,      0, 1]])

        self.X[0:3, 0] += R_theta * DeltaX

        # Jacobian
        Jr = np.mat(np.eye(3))
        Jr[0, 2] = -dx * sin_t - dy * cos_t
        Jr[1, 2] =  dx * cos_t - dy * sin_t

        self.P[0:3, 3:] = Jr @ self.P[0:3, 3:]
        
        self.P[3:, 0:3] = self.P[0:3, 3:].T

        Q_global = R_theta @ DeltaP @ R_theta.T
        self.P[0:3, 0:3] = (Jr @ self.P[0:3, 0:3] @ Jr.T) + Q_global

        # self.X[0:3,0] = ...
        # self.P[0:3,0:3] = ...
        self.lock.release()
        return (self.X,self.P)


    def update_ar(self, logger, Z, id, uncertainty):
            # Z = vstack([x,y])
            self.lock.acquire()
            
            logger.info(f"Update: Z={Z.T} X={self.X.T} Id={id}")
            
            R_cov = np.mat(np.diag([uncertainty, uncertainty]))
            theta = self.X[2, 0]
            cos_t, sin_t = cos(theta), sin(theta)
            
            R_theta = np.mat([[cos(theta), -sin(theta)],
                            [sin(theta),  cos(theta)]])

            if id not in self.idx:

                L_world = self.X[0:2, 0] + R_theta @ Z
                
                self.idx[id] = self.X.shape[0]
                self.X = np.vstack([self.X, L_world])
                
                # Jacobian after initialisation
                dx, dy = float(Z[0, 0]), float(Z[1, 0])
                G_robot = np.mat([[1, 0, -sin_t*dx - cos_t*dy],
                            [0, 1,  cos_t*dx - sin_t*dy]])
                
                size_old = self.P.shape[0]
                P_new = np.mat(np.zeros((size_old + 2, size_old + 2)))
                P_new[0:size_old, 0:size_old] = self.P
                
                P_new[size_old:, size_old:] = (G_robot @ self.P[0:3, 0:3] @ G_robot.T) + (R_theta @ R_cov @ R_theta.T)
                
                # Correlation between the new landmark and the robot pose
                P_new[size_old:, 0:size_old] = G_robot @ self.P[0:3, 0:size_old]
                P_new[0:size_old, size_old:] = P_new[size_old:, 0:size_old].T
                
                self.P = P_new

            else:
                l_idx = self.idx[id]
                
                # Z_hat = R_inv * (L_world - Robot_Pos)
                delta = self.X[l_idx:l_idx+2, 0] - self.X[0:2, 0]
                dx, dy = float(delta[0, 0]), float(delta[1, 0])
                
                Z_hat = R_theta.T @ delta
                
                # Jacobian H
                H = np.mat(np.zeros((2, self.X.shape[0])))
                
                H[0:2, 0:3] = np.mat([[-cos_t, -sin_t, -sin_t*dx + cos_t*dy],
                                [ sin_t, -cos_t, -cos_t*dx - sin_t*dy]])
                
                H[0:2, l_idx:l_idx+2] = R_theta.T 

                # Kalman gain and update
                S = (H @ self.P @ H.T) + R_cov
                K = self.P @ H.T @ inv(S)
                
                innovation = Z - Z_hat
                
                # update of state and covariance
                self.X += K @ innovation
                self.P = (np.mat(np.eye(self.P.shape[0])) - K @ H) @ self.P

            self.lock.release()
            return (self.X, self.P)

    def update_compass(self, logger, Z, uncertainty):
        self.lock.acquire()
        # TODO
        logger.info("Update: S="+str(Z)+" X="+str(self.X.T))
        # Update the full state self.X and self.P based on compass measurement
        # TODO
        H = np.zeros((1, self.X.shape[0]))
        H[0, 2] = 1.0  

        theta = self.X[2,0]
        y = Z - theta
        y = np.mat([(y + np.pi) % (2 * np.pi) - np.pi])

        #measurement noise
        R = np.mat([[uncertainty]])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        #update state and covariance
        self.X = self.X + K @ y
        self.P = (np.eye(self.X.shape[0]) - K @ H) @ self.P

        self.lock.release()
        return (self.X,self.P)


    def publish(self, pose_pub, odom_pub, target_frame, stamp, child_frame):
        pose = super().publish(pose_pub, odom_pub, target_frame, stamp, child_frame)
        ma = MarkerArray()
        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 5000
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = -0.1
        marker.scale.x = 3*sqrt(self.P[0,0])
        marker.scale.y = 3*sqrt(self.P[1,1]);
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        ma.markers.append(marker)
        for id in self.idx:
            marker = Marker()
            marker.header = pose.header
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            l = self.idx[id]
            marker.pose.position.x = self.X[l,0]
            marker.pose.position.y = self.X[l+1,0]
            marker.pose.position.z = -0.1
            marker.pose.orientation.x = 0.
            marker.pose.orientation.y = 0.
            marker.pose.orientation.z = 1.
            marker.pose.orientation.w = 0.
            marker.scale.x = 3*sqrt(self.P[l,l])
            marker.scale.y = 3*sqrt(self.P[l+1,l+1]);
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime = rclpy.time.Duration(seconds=3.).to_msg()
            ma.markers.append(marker)
            marker = Marker()
            marker.header = pose.header
            marker.ns = "landmark_kf"
            marker.id = 1000+id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = self.X[l+0,0]
            marker.pose.position.y = self.X[l+1,0]
            marker.pose.position.z = 1.0
            marker.pose.orientation.x = 0.
            marker.pose.orientation.y = 0.
            marker.pose.orientation.z = 1.
            marker.pose.orientation.w = 0.
            marker.text = str(id)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.lifetime = rclpy.time.Duration(seconds=3.).to_msg()
            ma.markers.append(marker)
        self.marker_pub.publish(ma)

