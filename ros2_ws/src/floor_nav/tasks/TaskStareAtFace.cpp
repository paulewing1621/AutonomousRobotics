#include <math.h>
#include "TaskStareAtFace.h"
#include "cs7630_msgs/msg/roi_array.hpp"

using namespace task_manager_msgs::msg;
using namespace task_manager_lib;
using namespace floor_nav;

TaskIndicator TaskStareAtFace::initialise()
{
    face_received_ = false;
    face_center_x = image_center_x;

    face_sub_ = node->create_subscription<cs7630_msgs::msg::ROIArray>(
        "/face_detect/rois", 10, std::bind(&TaskStareAtFace::faceCallback, this, std::placeholders::_1));
    return TaskStatus::TASK_INITIALISED;
}

void TaskStareAtFace::faceCallback(const cs7630_msgs::msg::ROIArray::SharedPtr msg)
{
    if (!msg->rois.empty()) {
        face_detected = true;
        auto roi = msg->rois[0];
        face_center_x = roi.x_offset + (roi.width / 2.0);
    }
    else {
        face_detected = false;
    }
}

TaskIndicator TaskStareAtFace::iterate()
{
    if (!face_detected) {
        env->publishVelocity(0.0, 0.0);
        return TaskStatus::TASK_RUNNING;
    }

    double error_pixels = image_center_x - face_center_x;
    double rot = cfg->k_theta * error_pixels;

    if (rot > cfg->max_angular_velocity)
        rot = cfg->max_angular_velocity;
    if (rot < -cfg->max_angular_velocity)
        rot = -cfg->max_angular_velocity;

    env->publishVelocity(0.0, rot);
    return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskStareAtFace::terminate()
{
    env->publishVelocity(0.0, 0.0);
    return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace)
