#include <math.h>
#include "TaskGoToPose.h"

using namespace task_manager_msgs::msg;
using namespace task_manager_lib;
using namespace floor_nav;

TaskIndicator TaskGoToPose::initialise() 
{
    RCLCPP_INFO(getNode()->get_logger(),"Going to %.2f %.2f %.2f (Smart: %s, Holonomic: %s)",
        cfg->goal_x, cfg->goal_y, cfg->goal_theta, 
        cfg->smart ? "YES" : "NO",
        cfg->holonomic ? "YES" : "NO");
        
    if (cfg->relative) {
        const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
        x_init = tpose.x;
        y_init = tpose.y;
        theta_init = tpose.theta;
    } else {
        x_init = 0.0;
        y_init = 0.0;
        theta_init = 0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskGoToPose::iterate()
{
    const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
    
    // 1. Calcul de la cible globale
    double target_x = x_init + cfg->goal_x;
    double target_y = y_init + cfg->goal_y;
    double target_theta = theta_init + cfg->goal_theta;

    // 2. Calcul des erreurs globales
    double dx = target_x - tpose.x;
    double dy = target_y - tpose.y;
    double theta_error = remainder(target_theta - tpose.theta, 2*M_PI);
    
    // Distance euclidienne
    double rho = hypot(dy, dx);
    
    // Variables pour le mode Non-Holonome (Smart/Dumb)
    double alpha = remainder(atan2(dy, dx) - tpose.theta, 2*M_PI);   
    double beta = remainder(target_theta - tpose.theta - alpha, 2*M_PI);

    // 3. Condition de fin
    if (rho < cfg->dist_threshold && fabs(theta_error) < cfg->angle_threshold) {
        return TaskStatus::TASK_COMPLETED;
    }

    double vel_x = 0.0;
    double vel_y = 0.0;
    double rot = 0.0;

    if (cfg->holonomic) {
        // --- MODE HOLONOME (CRABE / ROVER) ---
        // Le robot peut se déplacer latéralement.
        // Il faut projeter l'erreur globale (dx, dy) dans le repère LOCAL du robot.
        
        double c = cos(tpose.theta);
        double s = sin(tpose.theta);
        
        // Rotation : Monde -> Robot
        // dx_local (vel_x) est vers l'avant
        // dy_local (vel_y) est vers la gauche
        double dx_local = dx * c + dy * s;
        double dy_local = -dx * s + dy * c;
        
        // Commande Proportionnelle sur la position
        vel_x = cfg->k_v * dx_local;
        vel_y = cfg->k_v * dy_local;
        
        // Commande Proportionnelle sur l'angle (indépendante du mouvement)
        rot = cfg->k_alpha * theta_error;

        // Saturation de la vitesse linéaire totale (norme du vecteur vitesse)
        double v_total = hypot(vel_x, vel_y);
        if (v_total > cfg->max_velocity) {
            vel_x = (vel_x / v_total) * cfg->max_velocity;
            vel_y = (vel_y / v_total) * cfg->max_velocity;
        }
        
        // Saturation rotation
        if (rot > cfg->max_angular_velocity) rot = cfg->max_angular_velocity;
        if (rot < -cfg->max_angular_velocity) rot = -cfg->max_angular_velocity;

        // Publication avec 3 arguments (vx, vy, omega)
        env->publishVelocity(vel_x, vel_y, rot);
    }
    else if (cfg->smart) {
        // --- MODE SMART (Non-Holonome) ---
        if (rho > cfg->dist_threshold) {
            vel_x = cfg->k_v * rho; 
            rot = cfg->k_alpha * alpha + cfg->k_beta * beta;
        } else {
             // Alignement final
             vel_x = 0.0;
             rot = cfg->k_alpha * theta_error;
        }
        
        // Saturation
        if (vel_x > cfg->max_velocity) vel_x = cfg->max_velocity;
        if (vel_x < -cfg->max_velocity) vel_x = -cfg->max_velocity;
        if (rot > cfg->max_angular_velocity) rot = cfg->max_angular_velocity;
        if (rot < -cfg->max_angular_velocity) rot = -cfg->max_angular_velocity;

        // En mode non-holonome, vy est toujours 0
        env->publishVelocity(vel_x, 0.0, rot);
    } 
    else {
        // --- MODE DUMB (Non-Holonome) ---
        if (rho > cfg->dist_threshold) {
            // Etape 1: Tourner vers la cible 
            if (fabs(alpha) > M_PI/9) {
                rot = ((alpha > 0) ? 1 : -1) * cfg->max_angular_velocity;
                vel_x = 0.0;
            } else {
                // Etape 2: Aller vers la cible 
                vel_x = cfg->k_v * rho;
                rot = cfg->k_alpha * alpha; 
            }
        } 
        else {
            // Etape 3: Alignement final 
            vel_x = 0.0;
            rot = cfg->k_alpha * theta_error;
        }

        // Saturation
        if (vel_x > cfg->max_velocity) vel_x = cfg->max_velocity;
        if (vel_x < -cfg->max_velocity) vel_x = -cfg->max_velocity;
        if (rot > cfg->max_angular_velocity) rot = cfg->max_angular_velocity;
        if (rot < -cfg->max_angular_velocity) rot = -cfg->max_angular_velocity;

        // En mode non-holonome, vy est toujours 0
        env->publishVelocity(vel_x, 0.0, rot);
    }
    
    return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoToPose::terminate()
{
    env->publishVelocity(0,0,0);
    return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToPose);