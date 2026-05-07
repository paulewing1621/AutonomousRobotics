#!/usr/bin/env python3
import rclpy
import math
from task_manager_client_py.TaskClient import *

rclpy.init()
tc = TaskClient('/floor_tasks', 0.2)

scale = 0.5       # Taille du carré
vel = 0.1        # Vitesse max
my_kv = 1.5
my_ka = 2.0
my_kb = -1.5
threshold = 0.2

try:
    print("Début de la mission CARRE")

    # Premier coin 
    print("Go to Coin 1 (%.1f, 0.0)" % scale)
    tc.GoToPose(
        goal_x=scale, goal_y=0.0, goal_theta=math.pi/2, 
        smart=True, max_velocity=vel, dist_threshold=threshold,angle_threshold=threshold,
        k_v=my_kv, k_alpha=my_ka, k_beta=my_kb
    )

    #Deuxième coin 
    print("Go to Coin 2 (%.1f, %.1f)" % (scale, scale))
    tc.GoToPose(
        goal_x=scale, goal_y=scale, goal_theta=math.pi, 
        smart=True, max_velocity=vel, dist_threshold=threshold,angle_threshold=threshold,
        k_v=my_kv, k_alpha=my_ka, k_beta=my_kb
    )

    # Troisième coin 
    print("Go to Coin 3 (0.0, %.1f)" % scale)
    tc.GoToPose(
        goal_x=0.0, goal_y=scale, goal_theta=-math.pi/2, 
        smart=False, max_velocity=vel, dist_threshold=threshold,angle_threshold=threshold,
        k_v=my_kv, k_alpha=my_ka, k_beta=my_kb
    )

    # Retour départ 
    print("Retour Base (0.0, 0.0)")
    tc.GoToPose(
        goal_x=0.0, goal_y=0.0, goal_theta=0.0, 
        smart=False, max_velocity=vel, dist_threshold=threshold,angle_threshold=threshold,
        k_v=my_kv, k_alpha=my_ka, k_beta=my_kb
    )

    print("Mission terminée !")

except TaskException as e:
    print("Exception caught: " + str(e))
