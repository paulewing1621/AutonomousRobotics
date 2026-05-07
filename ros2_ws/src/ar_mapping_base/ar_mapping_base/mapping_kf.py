#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from numpy.linalg import inv
from math import pi, sin, cos, sqrt
from visualization_msgs.msg import Marker, MarkerArray
import threading

class Landmark:
    def __init__(self, Z, X, R):
        theta = X[2, 0]
        Rot = np.array([[cos(theta), -sin(theta)], 
                        [sin(theta),  cos(theta)]])
        
        self.L = X[0:2, 0] + Rot @ Z
        self.P = Rot @ R @ Rot.T

    def update(self, Z, X, R):
        theta = X[2, 0]
        dx = self.L[0, 0] - X[0, 0]
        dy = self.L[1, 0] - X[1, 0]

        Z_h = np.array([[ dx * cos(theta) + dy * sin(theta)],
                        [-dx * sin(theta) + dy * cos(theta)]])

        Rot = np.array([[ cos(theta), sin(theta)],
                      [-sin(theta), cos(theta)]])
        
        S = Rot @ self.P @ Rot.T + R
        K = self.P @ Rot.T @ inv(S)

        self.L = self.L + K @ (Z - Z_h)
        self.P = (np.eye(2) - K @ Rot) @ self.P        

class MappingKF:
    def __init__(self, node):
        self.lock = threading.Lock()
        self.marker_list = {}
        self.marker_pub = node.create_publisher(MarkerArray, "landmarks_kf", 10)

    def update_ar(self, logger, Z, X, Id, uncertainty):
        with self.lock:
            logger.info("Update: Z="+str(Z.T)+" X="+str(X.T)+" Id="+str(Id))
            R = np.diag([uncertainty, uncertainty])
            
            if Id in self.marker_list:
                self.marker_list[Id].update(Z, X, R)
            else:
                self.marker_list[Id] = Landmark(Z, X, R)
                logger.info(f"Initialised landmark {Id} at {self.marker_list[Id].L.T}")

    def publish(self, target_frame, timestamp):
        ma = MarkerArray()
        for idx, id_key in enumerate(self.marker_list):
            Lkf = self.marker_list[id_key]
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmarks"
            marker.id = int(id_key)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(Lkf.L[0, 0])
            marker.pose.position.y = float(Lkf.L[1, 0])
            marker.pose.position.z = 0.25 
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(2 * sqrt(abs(Lkf.P[0, 0])), 0.1)
            marker.scale.y = max(2 * sqrt(abs(Lkf.P[1, 1])), 0.1)
            marker.scale.z = 0.5
            marker.color.a = 0.8
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rclpy.time.Duration(seconds=1).to_msg()
            ma.markers.append(marker)
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "labels"
            text_marker.id = int(id_key) + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = marker.pose.position.x
            text_marker.pose.position.y = marker.pose.position.y
            text_marker.pose.position.z = 1.0
            text_marker.text = str(id_key)
            text_marker.scale.z = 0.3 
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            ma.markers.append(text_marker)
        self.marker_pub.publish(ma)