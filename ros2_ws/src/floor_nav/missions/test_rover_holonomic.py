#!/usr/bin/env python3
import rclpy
import math
from task_manager_client_py.TaskClient import *

rclpy.init()
tc = TaskClient('/floor_tasks', 0.2)

try:
    print("Test Holonomic (Déplacement en Crabe)")
    # Objectif : Aller en X=1, Y=1 (Diagonale)
    # Orientation finale : 0 (Le robot doit rester face à l'Est)
    
    # En mode normal, le robot tournerait à 45° avant d'avancer.
    # En mode holonome, il doit glisser latéralement.
    tc.GoToPose(
        goal_x=1.0, goal_y=1.0, goal_theta=0.0, 
        smart=False, 
        holonomic=True,   # <--- ACTIVATION DU MODE
        max_velocity=0.3,
        k_v=1.0, k_alpha=2.0
    )
    
    print("Mission terminée")

except TaskException as e:
    print("Exception caught: " + str(e))