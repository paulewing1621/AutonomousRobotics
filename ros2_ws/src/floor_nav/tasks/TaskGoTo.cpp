#include <math.h>
#include "TaskGoTo.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

TaskIndicator TaskGoTo::initialise() 
{
    RCLCPP_INFO(getNode()->get_logger(),"Going to %.2f %.2f (Holonomic: %s)",
        cfg->goal_x, cfg->goal_y, cfg->holonomic ? "YES" : "NO");
    
    if (cfg->relative) {
        const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
        x_init = tpose.x;
        y_init = tpose.y;
    } else {
        x_init = 0.0;
        y_init = 0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskGoTo::iterate()
{
    const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();

    double target_x = x_init + cfg->goal_x;
    double target_y = y_init + cfg->goal_y;

    double dx = target_x - tpose.x;
    double dy = target_y - tpose.y;
    double r = hypot(dy, dx);

    if (r < cfg->dist_threshold) {
        return TaskStatus::TASK_COMPLETED;
    }

    if (cfg->holonomic) {
        double c = cos(tpose.theta);
        double s = sin(tpose.theta);
        
        double vx_local = dx * c + dy * s;
        double vy_local = -dx * s + dy * c;
        
        double vel_x = cfg->k_v * vx_local;
        double vel_y = cfg->k_v * vy_local;

        double v_total = hypot(vel_x, vel_y);
        if (v_total > cfg->max_velocity) {
            vel_x = (vel_x / v_total) * cfg->max_velocity;
            vel_y = (vel_y / v_total) * cfg->max_velocity;
        }
        env->publishVelocity(vel_x, vel_y, 0.0);
    } 
    else {
        double alpha = remainder(atan2(dy, dx) - tpose.theta, 2*M_PI);
        if (fabs(alpha) > M_PI/9) {
            double rot = ((alpha > 0) ? 1 : -1) * cfg->max_angular_velocity;
            env->publishVelocity(0.0, 0.0, rot);
        } else {
            double vel = cfg->k_v * r;
            double rot = std::max(std::min(cfg->k_alpha * alpha, cfg->max_angular_velocity), -cfg->max_angular_velocity);
            
            if (vel > cfg->max_velocity) vel = cfg->max_velocity;
            env->publishVelocity(vel, 0.0, rot);
        }
    }
    return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoTo::terminate()
{
    env->publishVelocity(0, 0, 0);
    return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoTo);