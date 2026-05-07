#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from numpy import mat, vstack, multiply, zeros, random, exp
from numpy.linalg import inv
from math import pi, sin, cos, atan2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import bisect
import threading
from rover_driver_base.rover_kinematics import RoverKinematics
from ar_loc_base.rover_odo import RoverOdo, DeltaOdo

class DeltaPF(DeltaOdo):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node, initial_pose, initial_uncertainty)
        self.N = 500
        self.particles = [self.X + self.drawNoise(initial_uncertainty) for i in range(0, self.N)]
        self.pa_pub = node.create_publisher(PoseArray, "~/particles", 1)

    def getRotationFromWorldToRobot(self):
        return self.getRotation(-self.X[2, 0])

    def drawNoise(self, norm):
        if isinstance(norm, list):
            return mat(vstack(norm) * (2 * random.rand(len(norm), 1) - vstack([1] * len(norm))))
        else:
            return mat(multiply(norm, ((2 * random.rand(3, 1) - vstack([1, 1, 1])))))

    def applyDisplacement(self, X, DeltaX, Uncertainty):
        noise = self.drawNoise(Uncertainty)
        noisy_delta = DeltaX + noise

        theta = X[2, 0]
        rot_mot = mat([[cos(theta), -sin(theta), 0],
                       [sin(theta),  cos(theta), 0],
                       [0,           0,          1]])

        return X + rot_mot * noisy_delta

    def predict_delta(self, logger, DeltaX, Uncertainty, lock=True):
        if lock:
            self.lock.acquire()

        if hasattr(Uncertainty, 'shape'):
            if Uncertainty.shape == (3, 1):
                noise = Uncertainty
            elif Uncertainty.shape == (3, 3):
                noise = np.diag(Uncertainty).reshape((3, 1))
            elif len(Uncertainty.shape) > 1:
                noise = np.diag(Uncertainty).reshape((3, 1))
            else:
                noise = Uncertainty.reshape((3, 1))
        else:
            noise = np.array([[Uncertainty, Uncertainty, Uncertainty]]).T

        new_particles = []
        for p in self.particles:
            new_particles.append(self.applyDisplacement(p, DeltaX, noise))
        
        self.particles = new_particles
        self.updateMean(logger)

        if lock:
            self.lock.release()

    def evalParticleAR(self, X, Z, L, Uncertainty):
        dx_world = L[0, 0] - X[0, 0]
        dy_world = L[1, 0] - X[1, 0]
        theta = X[2, 0]
        c = cos(theta)
        s = sin(theta)
        
        x_pred = c * dx_world + s * dy_world
        y_pred = -s * dx_world + c * dy_world
        
        dist_sq = (Z[0, 0] - x_pred)**2 + (Z[1, 0] - y_pred)**2
        
        sigma = max(float(Uncertainty), 0.01)
        weight = exp(-dist_sq / (2 * sigma**2))
        return weight

    def evalParticleCompass(self, X, Value, Uncertainty):
        theta_particle = X[2, 0]
        diff = atan2(sin(Value - theta_particle), cos(Value - theta_particle))
        sigma = max(float(Uncertainty), 0.01)
        weight = exp(-(diff**2) / (2 * sigma**2))
        return weight

    def update_ar(self, logger, Z, L, Uncertainty):
        self.lock.acquire()
        logger.info("Update AR: L=" + str(L.T) + " X=" + str(self.X.T))

        weights = []
        for p in self.particles:
            w = self.evalParticleAR(p, Z, L, Uncertainty)
            weights.append(w)

        sum_weights = sum(weights)
        if sum_weights == 0:
            weights = [1.0 / self.N] * self.N
        else:
            weights = [w / sum_weights for w in weights]

        indices = np.random.choice(self.N, size=self.N, p=weights)
        self.particles = [self.particles[i] for i in indices]
        
        self.updateMean(logger)
        self.lock.release()

    def update_compass(self, logger, angle, Uncertainty):
        self.lock.acquire()
        logger.info("Update Compass: S=" + str(angle) + " X=" + str(self.X.T))

        weights = []
        for p in self.particles:
            w = self.evalParticleCompass(p, angle, Uncertainty)
            weights.append(w)

        sum_weights = sum(weights)
        if sum_weights == 0:
            weights = [1.0 / self.N] * self.N
        else:
            weights = [w / sum_weights for w in weights]

        indices = np.random.choice(self.N, size=self.N, p=weights)
        self.particles = [self.particles[i] for i in indices]
        
        self.updateMean(logger)
        self.lock.release()

    def updateMean(self, logger):
        X_sum = mat(zeros((4, 1)))
        for x in self.particles:
            y = np.mat([[x[0, 0], x[1, 0], np.cos(x[2, 0]), np.sin(x[2, 0])]]).T
            X_sum += y
        
        X_mean = X_sum / len(self.particles)
        self.X = np.mat([[X_mean[0, 0], X_mean[1, 0], atan2(X_mean[3, 0], X_mean[2, 0])]]).T
        return self.X

    def publish(self, pose_pub, odom_pub, target_frame, stamp, child_frame):
        pose = super().publish(pose_pub, odom_pub, target_frame, stamp, child_frame)

        pa = PoseArray()
        pa.header = pose.header
        for p in self.particles:
            po = Pose()
            po.position.x = p[0, 0]
            po.position.y = p[1, 0]
            po.position.z = 0.0
            q = self.quaternion_from_euler(0, 0, p[2, 0])
            po.orientation.x = q[0]
            po.orientation.y = q[1]
            po.orientation.z = q[2]
            po.orientation.w = q[3]
            pa.poses.append(po)
        self.pa_pub.publish(pa)


class RoverPF(DeltaPF):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node, initial_pose, initial_uncertainty)
        self.kinematics = RoverKinematics()

    def predict(self, logger, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        if self.first_run:
            self.kinematics.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return

        iW = self.kinematics.prepare_inversion_matrix(drive_cfg)
        S = self.kinematics.prepare_displacement_matrix(self.kinematics.motor_state, motor_state, drive_cfg)
        self.kinematics.motor_state.copy(motor_state)
        
        DeltaX = iW @ S
        Uncertainty = iW @ mat(vstack([encoder_precision] * len(S)))

        self.predict_delta(logger, DeltaX, Uncertainty, False)
        self.lock.release()