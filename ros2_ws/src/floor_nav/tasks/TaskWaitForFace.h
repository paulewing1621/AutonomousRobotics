#ifndef TASK_WAIT_FOR_FACE_H
#define TASK_WAIT_FOR_FACE_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"
#include "cs7630_msgs/msg/roi_array.hpp"

using namespace task_manager_lib;

namespace floor_nav {
    struct TaskWaitForFaceConfig : public TaskConfig {
        TaskWaitForFaceConfig() {}
    };

    class TaskWaitForFace : public TaskInstance<TaskWaitForFaceConfig, SimTasksEnv>
    {
        protected:
            bool face_detected;
            rclcpp::Subscription<cs7630_msgs::msg::ROIArray>::SharedPtr face_sub;
            void faceCallback(const cs7630_msgs::msg::ROIArray::SharedPtr msg);
    
        public:
            TaskWaitForFace(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def, env) {}
            virtual ~TaskWaitForFace() {}
            virtual TaskIndicator initialise();
            virtual TaskIndicator iterate();
            virtual TaskIndicator terminate();
    };

    class TaskFactoryWaitForFace : public TaskDefinition<TaskWaitForFaceConfig, SimTasksEnv, TaskWaitForFace>
    {
        public:
            TaskFactoryWaitForFace(TaskEnvironmentPtr env) : 
                Parent("WaitForFace", "Do nothing until a face is detected", true, env) {}
            virtual ~TaskFactoryWaitForFace() {}
    };

} 

#endif 
