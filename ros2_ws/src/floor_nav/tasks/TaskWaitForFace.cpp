#include "TaskWaitForFace.h"
#include "cs7630_msgs/msg/roi_array.hpp"

using namespace task_manager_msgs::msg;
using namespace task_manager_lib;
using namespace floor_nav;

TaskIndicator TaskWaitForFace::initialise()
{
    face_detected = false;
    face_sub = node->create_subscription<cs7630_msgs::msg::ROIArray>(
    "/face_detect/rois", 10, std::bind(&TaskWaitForFace::faceCallback, this, std::placeholders::_1));
    return TaskStatus::TASK_INITIALISED;
}

void TaskWaitForFace::faceCallback(const cs7630_msgs::msg::ROIArray::SharedPtr msg)
{
    if (!msg->rois.empty()) {
        face_detected = true;
    }
}

TaskIndicator TaskWaitForFace::iterate()
{
    if (face_detected) {
        return TaskStatus::TASK_COMPLETED;
    }
    return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskWaitForFace::terminate()
{
    face_sub.reset(); 
    
    return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryWaitForFace)
