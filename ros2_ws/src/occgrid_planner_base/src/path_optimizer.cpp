
#include <vector>
#include <string>
#include <map>
#include <list>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <cs7630_msgs/msg/trajectory.hpp>
#include <cs7630_msgs/msg/trajectory_element.hpp>
using std::placeholders::_1;
using namespace std::chrono_literals;
class PathOptimizer : public rclcpp::Node {
    protected:
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Publisher<cs7630_msgs::msg::Trajectory>::SharedPtr traj_pub_;
        double velocity_;
        double max_acceleration_;
        double max_braking_;
        void path_cb(nav_msgs::msg::Path::SharedPtr msg) {
            std::vector<float> omega(msg->poses.size(),0.0);
            std::vector<float> s(msg->poses.size(),0.0);
            std::vector<float> heading(msg->poses.size(),0.0);
            for (unsigned i=1;i<msg->poses.size();i++) {
                const geometry_msgs::msg::Point& P0 = msg->poses[i-1].pose.position;
                const geometry_msgs::msg::Point& P1 = msg->poses[i].pose.position;
                double ds = hypot(P1.y-P0.y,P1.x-P0.x);
                double dt = ds / velocity_;
                heading[i] = atan2(P1.y-P0.y,P1.x-P0.x);
                s[i] = s[i-1] + ds;
                omega[i] = remainder(heading[i]-heading[i-1], 2*M_PI) / dt;
            }
            if (heading.size()>1) {
                heading[0] = heading[1];
            }
            cs7630_msgs::msg::Trajectory output;
            output.header = msg->header;
            output.ts.resize(msg->poses.size());
            for (unsigned i=0;i<msg->poses.size();i++) {
                output.ts[i].header = msg->poses[i].header;
                output.ts[i].header.stamp = rclcpp::Time(output.header.stamp) 
                    + rclcpp::Duration(std::chrono::duration<double>(s[i]/velocity_));
                output.ts[i].pose.position = msg->poses[i].pose.position;
                tf2::Quaternion q;
                q.setRPY(0,0,heading[i]);
                output.ts[i].pose.orientation = tf2::toMsg(q);
                output.ts[i].twist.linear.x = velocity_;
                output.ts[i].twist.angular.z = omega[i];
            }
            if (output.ts.size()>0) {
                unsigned int j = output.ts.size() - 1;
                output.ts[j].twist.linear.x = 0;
                output.ts[j].twist.angular.z = 0;
            }
            traj_pub_->publish(output);
            RCLCPP_INFO(this->get_logger(),"Optimized path into a trajectory");
        }

    public:
        PathOptimizer() : rclcpp::Node("path_optimizer") {
            this->declare_parameter("~/velocity",1.0);
            this->declare_parameter("~/max_acceleration",1.0);
            this->declare_parameter("~/max_braking",1.0);
            velocity_ = this->get_parameter("~/velocity").as_double();
            max_acceleration_ = this->get_parameter("~/max_acceleration").as_double();
            max_braking_ = this->get_parameter("~/max_braking").as_double();
            path_sub_ = this->create_subscription<nav_msgs::msg::Path>("~/path",1,
                    std::bind(&PathOptimizer::path_cb,this,std::placeholders::_1));
            traj_pub_ = this->create_publisher<cs7630_msgs::msg::Trajectory>("~/trajectory",1);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathOptimizer>());
    rclcpp::shutdown();
}



