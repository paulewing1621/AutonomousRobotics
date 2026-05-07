#ifndef TASK_STARE_AT_FACE_H
#define TASK_STARE_AT_FACE_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"
#include "cs7630_msgs/msg/roi_array.hpp"

using namespace task_manager_lib;

namespace floor_nav {

    struct TaskStareAtFaceConfig : public TaskConfig {
        TaskStareAtFaceConfig() {
            k_theta = 0.005; 
            max_angular_velocity = 1.0;
        }
        double k_theta;
        double max_angular_velocity;
    };

    class TaskStareAtFace : public TaskInstance<TaskStareAtFaceConfig, SimTasksEnv>
    {
    protected:
        bool face_detected;
        bool face_received_;
        double face_center_x;
        double image_center_x = 320.0; 


        rclcpp::Subscription<cs7630_msgs::msg::ROIArray>::SharedPtr face_sub_;
        
        void faceCallback(const cs7630_msgs::msg::ROIArray::SharedPtr msg);
        
    public:
        TaskStareAtFace(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def, env) {}
        virtual ~TaskStareAtFace() {};

        virtual TaskIndicator initialise();
        virtual TaskIndicator iterate();
        virtual TaskIndicator terminate();
    };

    class TaskFactoryStareAtFace : public TaskDefinition<TaskStareAtFaceConfig, SimTasksEnv, TaskStareAtFace>
    {
    public:
        TaskFactoryStareAtFace(TaskEnvironmentPtr env) :
            Parent("StareAtFace", "Rotate robot to center a detected face in the image", true, env) {}
        virtual ~TaskFactoryStareAtFace() {};
    };
}

#endif 