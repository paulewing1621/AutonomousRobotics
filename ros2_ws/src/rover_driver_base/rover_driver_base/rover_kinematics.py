#!/usr/bin/env python
from geometry_msgs.msg import Twist
import numpy as np
from numpy.linalg import pinv
from math import atan2, hypot, pi, cos, sin, tan, atan

prefix=["FL","FR","CL","CR","RL","RR"]

class RoverMotors:
    def __init__(self):
        self.steering={}
        self.drive={}
        for k in prefix:
            self.steering[k]=0.0
            self.drive[k]=0.0
    def copy(self,value):
        for k in prefix:
            self.steering[k]=value.steering[k]
            self.drive[k]=value.drive[k]

class DriveConfiguration:
    def __init__(self,radius,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius


class RoverKinematics:
    def __init__(self):
        self.X = np.asmatrix(np.zeros((3,1)))
        self.motor_state = RoverMotors()
        self.ICR = (pi/2,pi/2)
        self.first_run = True

    @staticmethod
    def ICR_cart_to_polar(x,y):
        return (atan2(y,x), atan(hypot(y,x)))

    @staticmethod
    def ICR_polar_to_cart(theta,phi):
        r = tan(phi)
        return (r*cos(theta),r*sin(theta))

    @staticmethod
    def ICR_from_twist(vx,vy,wz):
        theta=atan2(vy,vx)+pi/2
        phi=atan2(hypot(vx,vy),wz)
        return (theta,phi)

    @staticmethod
    def ICR_to_twist(theta,phi,v):
        r = tan(phi)
        T = Twist()
        T.angular.z = v / r
        T.linear.x = v * cos(theta-pi/2)
        T.linear.y = v * sin(theta-pi/2)
        return T

    def filter_twist(self, twist_in, drive_cfg):
        vx = twist_in.linear.x; vy = twist_in.linear.y; wz = twist_in.angular.z
        v = hypot(vx,vy)
        if abs(v)<1e-2:
            # Maintain ICR while stopped to avoid jerky steering jumps
            theta, phi = self.ICR
            return RoverKinematics.ICR_to_twist(theta, phi, 1e-3)
        
        # Update current ICR for next call
        self.ICR = RoverKinematics.ICR_from_twist(vx, vy, wz)
        return twist_in

    def twist_to_motors(self, twist, drive_cfg, skidsteer=False, drive_state=None):
        motors = RoverMotors()
        vx = twist.linear.x
        vy = twist.linear.y
        wz = twist.angular.z

        if skidsteer:
            for k in drive_cfg.keys():
                cfg = drive_cfg[k]
                motors.steering[k] = 0.0
                # Tank-drive style: angular velocity creates speed difference based on y-offset
                motors.drive[k] = (vx - wz * cfg.y) / cfg.radius
        else:
            for k in drive_cfg.keys():
                cfg = drive_cfg[k]
                # Calculate the linear velocity vector for each specific wheel
                v_ix = vx - wz * cfg.y
                v_iy = vy + wz * cfg.x
                # atan2 gives the steering angle; hypot gives the magnitude (speed)
                motors.steering[k] = atan2(v_iy, v_ix)
                motors.drive[k] = hypot(v_ix, v_iy) / cfg.radius
        return motors

    def prepare_inversion_matrix(self, drive_cfg):
        # W maps Robot Twist [vx, vy, wz] to Wheel Velocities [v_ix, v_iy]
        W = np.zeros((2 * len(drive_cfg), 3))
        for i, k in enumerate(prefix):
            cfg = drive_cfg[k]
            # Row for v_ix
            W[2*i, 0] = 1.0
            W[2*i, 1] = 0.0
            W[2*i, 2] = -cfg.y
            # Row for v_iy
            W[2*i+1, 0] = 0.0
            W[2*i+1, 1] = 1.0
            W[2*i+1, 2] = cfg.x
        return pinv(W)

    def prepare_displacement_matrix(self, motor_state_t1, motor_state_t2, drive_cfg):
        # S contains the x and y displacement components of each wheel
        S = np.zeros((2 * len(drive_cfg), 1))
        for i, k in enumerate(prefix):
            cfg = drive_cfg[k]
            
            # Encoder difference in radians
            d_phi = motor_state_t2.drive[k] - motor_state_t1.drive[k]

            # Handle encoder wrap-around at pi / -pi
            while d_phi > pi: d_phi -= 2*pi
            while d_phi < -pi: d_phi += 2*pi

            dist = d_phi * cfg.radius
            alpha = motor_state_t2.steering[k]
            
            # Project wheel distance into local X and Y
            S[2*i, 0] = dist * cos(alpha)
            S[2*i+1, 0] = dist * sin(alpha)
        return S

    def compute_displacement(self, motor_state, drive_cfg):
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            return np.asmatrix(np.zeros((3,1)))

        # Solve the system S = W * dX using the pseudo-inverse
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state, motor_state, drive_cfg)
        
        dX = np.asmatrix(iW) * np.asmatrix(S)
        
        self.motor_state.copy(motor_state)
        return dX

    def integrate_odometry(self, motor_state, drive_cfg):
        # Local displacement: dX[0]=dx, dX[1]=dy, dX[2]=dtheta
        dX = self.compute_displacement(motor_state, drive_cfg)
        
        dx_local = dX[0, 0]
        dy_local = dX[1, 0]
        dtheta = dX[2, 0]
        
        # Current global orientation
        theta = self.X[2, 0]
        
        # Transform local displacement to global frame using rotation matrix
        # x_global = x_local * cos(theta) - y_local * sin(theta)
        # y_global = x_local * sin(theta) + y_local * cos(theta)
        self.X[0, 0] += dx_local * cos(theta) - dy_local * sin(theta)
        self.X[1, 0] += dx_local * sin(theta) + dy_local * cos(theta)
        self.X[2, 0] += dtheta
        
        # Normalize theta to [-pi, pi] for stability
        self.X[2, 0] = atan2(sin(self.X[2, 0]), cos(self.X[2, 0]))
        
        return self.X

def quaternion_from_euler(ai, aj, ak):
    # Standard Euler to Quaternion conversion for ROS2 (Yaw only mostly used here)
    ai /= 2.0; aj /= 2.0; ak /= 2.0
    ci = cos(ai); si = sin(ai)
    cj = cos(aj); sj = sin(aj)
    ck = cos(ak); sk = sin(ak)

    q = np.empty((4, ))
    q[0] = si * cj * ck - ci * sj * sk
    q[1] = ci * sj * ck + si * cj * sk
    q[2] = ci * cj * sk - si * sj * ck
    q[3] = ci * cj * ck + si * sj * sk

    return q