#include <vector>
#include <string>
#include <map>
#include <list>
#include <chrono>
#include <functional>
#include <cmath>
#include <utility>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00
#define WIN_SIZE 800
using std::placeholders::_1;
using namespace std::chrono_literals;

class OccupancyGridPlanner : public rclcpp::Node {
    protected:
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr og_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr optimal_target_pub_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr explorer_service_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer_exploration_;

        // Runtime behavior control
        bool do_explore_{false};
        cv::Point2i current_target_;
        bool has_current_target_{false};
        static constexpr float GOAL_REACHED_THRESH = 5.0f; // cells (~50cm)


        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;

        cv::Rect roi_;
        cv::Mat_<uint8_t> og_, cropped_og_,og_without_unknown; 
        cv::Mat_<bool> frontier_points;
        std::vector<cv::Point2i> frontier_points_vector;
        cv::Mat_<cv::Vec3b> og_rgb_, og_rgb_marked_;
        cv::Point og_center_;
        nav_msgs::msg::MapMetaData info_;
        std::string frame_id_;
        std::string base_link_;
        bool headless_;
        bool ready_;
        bool debug_;
        double robot_radius_;

        typedef std::multimap<float, cv::Point3i> Heap;

        // Example of code to convert between Point3i and Point2i, aka Point
        cv::Point P2(const cv::Point3i & P) {return cv::Point(P.x,P.y);}

        unsigned int angle_to_index(double yaw) {
            // Normalize to 0..2PI
            double angle = fmod(yaw, 2 * M_PI);
            if (angle < 0) angle += 2 * M_PI;
            // The sector 0 should be centered on 0.
            unsigned int idx = (unsigned int)(round(angle * 8.0 / (2.0 * M_PI))) % 8;
            return idx;
        }

        double index_to_angle(unsigned int idx) {
            return idx * 2.0 * M_PI / 8.0;
        }

        // Callback for Occupancy Grids
        void og_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            info_ = msg->info;
            frame_id_ = msg->header.frame_id;
            // Create an image to store the value of the grid.
            og_ = cv::Mat_<uint8_t>(msg->info.height, msg->info.width,0xFF);

            og_without_unknown = cv::Mat_<uint8_t>(msg->info.height, msg->info.width,0xFF);
            og_center_ = cv::Point(-info_.origin.position.x/info_.resolution,
                    -info_.origin.position.y/info_.resolution);

            // Some variables to select the useful bounding box 
            unsigned int maxx=0, minx=msg->info.width, 
                         maxy=0, miny=msg->info.height;
            // Convert the representation into something easy to display.
            for (unsigned int j=0;j<msg->info.height;j++) {
                for (unsigned int i=0;i<msg->info.width;i++) {
                    int8_t v = msg->data[j*msg->info.width + i];
                    switch (v) {
                        case 0: 
                            og_(j,i) = FREE; 
                            og_without_unknown(j,i) = FREE;
                            break;
                        case 100: 
                            og_(j,i) = OCCUPIED; 
                            og_without_unknown(j,i) = OCCUPIED;
                            break;
                        case -1: 
                        default:
                            og_(j,i) = UNKNOWN; 
                            og_without_unknown(j,i) = FREE;
                            break;
                    }
                    // Update the bounding box of free or occupied cells.
                    if (og_(j,i) != UNKNOWN) {
                        minx = std::min(minx,i);
                        miny = std::min(miny,j);
                        maxx = std::max(maxx,i);
                        maxy = std::max(maxy,j);
                    }
                }
            }
            // TODO: Implement obstacle expansion here
            // -----------------------

            // STEP 1
            // convert robot radius to pixels
            int robot_radius_px = std::max(1, (int)(robot_radius_ / info_.resolution));
            
            // create a circular element to represent the robot's footprint
            cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * robot_radius_px + 1, 2 * robot_radius_px + 1));


            cv::Mat temp_og; // temporary
            cv::Mat temp_og_without_unknown; // temporary
            cv::bitwise_not(og_, temp_og); //invert grid values because dilate expand high values
            cv::bitwise_not(og_without_unknown, temp_og_without_unknown); 


            
            cv::dilate(temp_og_without_unknown, temp_og_without_unknown, element);

            

            cv::bitwise_or(temp_og_without_unknown,temp_og, temp_og);
            

            cv::bitwise_not(temp_og, og_); //invert back

            if (!ready_) {
                ready_ = true;
                RCLCPP_INFO(this->get_logger(),"Received occupancy grid, ready_ to plan");
            }

            // The lines below are only for display
            unsigned int w = maxx - minx;
            unsigned int h = maxy - miny;
            roi_ = cv::Rect(minx,miny,w,h);
            if (!headless_) {
                cv::cvtColor(og_, og_rgb_, cv::COLOR_GRAY2RGB);
                // Compute a sub-image that covers only the useful part of the
                // grid.
                cropped_og_ = cv::Mat_<uint8_t>(og_,roi_);
                if ((w > WIN_SIZE) || (h > WIN_SIZE)) {
                    // The occupancy grid is too large to display. We need to scale
                    // it first.
                    double ratio = w / ((double)h);
                    cv::Size new_size;
                    if (ratio >= 1) {
                        new_size = cv::Size(WIN_SIZE,WIN_SIZE/ratio);
                    } else {
                        new_size = cv::Size(WIN_SIZE*ratio,WIN_SIZE);
                    }
                    cv::Mat_<uint8_t> resized_og;
                    cv::resize(cropped_og_,resized_og,new_size);
                    cv::imshow( "OccGrid", resized_og );
                } else {
                    cv::imshow( "OccGrid", cropped_og_ );
                    cv::imshow( "OccGrid", og_rgb_ );
                }
            }


            // create_frontier_points(og_, msg->info.height, msg->info.width);

        }
        
        void exploration_step(bool new_step=true) {
            if (do_explore_) {
                if (new_step) {
                    int height = og_.rows;
                    int width = og_.cols;
                    create_frontier_points(og_, height, width);
                }
                if (frontier_points_vector.empty()) {
                    RCLCPP_INFO(this->get_logger(), "No frontier points available — waiting for map update");
                    return;
                }
                try {
                    geometry_msgs::msg::TransformStamped transformStamped;
                    transformStamped = tf_buffer->lookupTransform(frame_id_, base_link_, tf2::TimePointZero);

                    double s_yaw = tf2::getYaw(transformStamped.transform.rotation);
                    cv::Point2i start_2d = cv::Point2i(transformStamped.transform.translation.x / info_.resolution,
                            transformStamped.transform.translation.y / info_.resolution) + og_center_;
                    cv::Point3i start = cv::Point3i(start_2d.x, start_2d.y, angle_to_index(s_yaw));

                    // Keep current target if robot hasn't reached it yet and it's still a frontier
                    if (has_current_target_ && new_step) {
                        float dist_to_target = std::hypot(start_2d.x - current_target_.x,
                                                          start_2d.y - current_target_.y);
                        bool target_still_valid = std::find(frontier_points_vector.begin(),
                                                            frontier_points_vector.end(),
                                                            current_target_) != frontier_points_vector.end();
                        if (dist_to_target > GOAL_REACHED_THRESH && target_still_valid) {
                            RCLCPP_DEBUG(this->get_logger(), "Still heading to current target (%.1f cells away) — not replanning", dist_to_target);
                            cv::Point3i current_target_3d(current_target_.x, current_target_.y, angle_to_index(0));
                            planToPixelTarget(start, current_target_3d);
                            return;
                        }
                        has_current_target_ = false;
                    }

                    cv::Point2i optimal_target_2d = selectOptimalPoint(start);
                    current_target_ = optimal_target_2d;
                    has_current_target_ = true;
                    cv::Point3i optimal_target = cv::Point3i(optimal_target_2d.x, optimal_target_2d.y, angle_to_index(0));

                    planToPixelTarget(start, optimal_target);

                } catch (const tf2::TransformException & ex) {
                    RCLCPP_DEBUG(this->get_logger(), "Cannot get robot position: %s", ex.what());
                }
            }
        }

        void create_frontier_points(cv::Mat_<uint8_t> og_, int height, int width) {

            frontier_points = cv::Mat_<bool>(height, width, false);
            frontier_points_vector.clear();

            for (int j=1;j<height-1;j++) {
                for (int i=1;i<width-1;i++) {
                    if (og_(j,i) == FREE) {
                        if (og_(j-1,i) == UNKNOWN || og_(j+1,i) == UNKNOWN || og_(j,i-1) == UNKNOWN || og_(j,i+1) == UNKNOWN) {
                            frontier_points(j,i) = true;
                            frontier_points_vector.emplace_back(j, i);
                        }
                    }
                }
            }
            if (!headless_) {
                cv::imshow( "frontier", frontier_points *255);
            }
        }



        float computeDistanceCost(const cv::Point& a, const cv::Point& b) {
            return std::hypot(a.x - b.x, a.y - b.y);
        }


        int computeInformationGain(const cv::Point& p, int r = 5) {
            int gain = 0;
            for (int dy = -r; dy <= r; dy++) {
                for (int dx = -r; dx <= r; dx++) {
                    cv::Point neighbor = p + cv::Point(dx, dy);
                    if (!isInGrid(neighbor)) continue;
                    if (og_(neighbor.y, neighbor.x) == UNKNOWN) gain++;
                }
            }
            return gain;
        }

        // Count occupied cells within radius r — higher means closer to a wall
        int computeWallProximity(const cv::Point& p, int r = 2) {
            int wall_count = 0;
            for (int dy = -r; dy <= r; dy++) {
                for (int dx = -r; dx <= r; dx++) {
                    cv::Point neighbor = p + cv::Point(dx, dy);
                    if (!isInGrid(neighbor)) continue;
                    if (og_without_unknown(neighbor.y, neighbor.x) == OCCUPIED) wall_count++;
                }
            }
            return wall_count;
        }

        cv::Point2i selectOptimalPoint(const cv::Point3i& robot_position) {

            if (frontier_points_vector.empty()) {
                RCLCPP_INFO(this->get_logger(), "No frontier points available");
                return cv::Point2i(robot_position.x, robot_position.y);
            }

            cv::Point2i best_point = frontier_points_vector[0];
            float best_score = -1e9f;

            for (const cv::Point2i& p : frontier_points_vector) {

                if (og_(p.y, p.x) != FREE) {
                    continue;
                }

                float dist = std::hypot(p.x - robot_position.x, p.y - robot_position.y);
                int gain = computeInformationGain(p);
                int wall_proximity = computeWallProximity(p);

                float distance_weight = 2.0;
                float gain_weight = 1.0;
                float wall_weight = 1.3;

                float score = distance_weight * dist + gain_weight * gain - wall_weight * wall_proximity;

                if (score > best_score) {
                    best_score = score;
                    best_point = p;
                }
            }

            return best_point;
        }

        // Generic test if a point is within the occupancy grid
        bool isInGrid(const cv::Point & P) {
            if ((P.x < 0) || (P.x >= (signed)info_.width) 
                    || (P.y < 0) || (P.y >= (signed)info_.height)) {
                return false;
            }
            return true;
        }

        // This is called when a new goal is posted by RViz. We don't use a
        // mutex here, because it can only be called in spinOnce.
        void target_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            geometry_msgs::msg::TransformStamped transformStamped;
            geometry_msgs::msg::PoseStamped pose;
            if (!ready_) {
                RCLCPP_WARN(this->get_logger(),"Ignoring target while the occupancy grid has not been received");
                return;
            }
            RCLCPP_INFO(this->get_logger(),"Received planning request in frame %s (base link %s, frame id %s)",
                    msg->header.frame_id.c_str(),
                    base_link_.c_str(),
                    frame_id_.c_str());
            og_rgb_marked_ = og_rgb_.clone();
            // Convert the destination point in the occupancy grid frame. 
            // The debug case is useful is the map is published without
            // gmapping running (for instance with map_server).
            if (debug_) {
                pose = *msg;
            } else {
                try{
                    std::string errStr;
                    // This converts target in the grid frame.
                    if (!tf_buffer->canTransform(frame_id_, msg->header.frame_id, msg->header.stamp,
                                rclcpp::Duration(std::chrono::duration<double>(1.0)),&errStr)) {
                        RCLCPP_ERROR(this->get_logger(),"Cannot transform target: %s",errStr.c_str());
                        return;
                    }
                    transformStamped = tf_buffer->lookupTransform(frame_id_, msg->header.frame_id, msg->header.stamp);
                    tf2::doTransform(*msg,pose,transformStamped);

                    // this gets the robot pose at the same time as the goal to ensure alignment
                    if (!tf_buffer->canTransform(frame_id_, base_link_, msg->header.stamp,
                                rclcpp::Duration(std::chrono::duration<double>(1.0)),&errStr)) {
                        RCLCPP_ERROR(this->get_logger(),"Cannot transform base_link: %s",errStr.c_str());
                        return;
                    }
                    transformStamped = tf_buffer->lookupTransform(frame_id_, base_link_, msg->header.stamp);
                }
                catch (const tf2::TransformException & ex){
                    RCLCPP_ERROR(this->get_logger(),"%s",ex.what());
                    return;
                }
            }
            // Now scale the target to the grid resolution and shift it to the
            // grid center.
            // For reference, this recovers the robot orientation
            double t_yaw = tf2::getYaw(pose.pose.orientation);
            cv::Point2i target_2d = cv::Point2i(pose.pose.position.x / info_.resolution, 
                    pose.pose.position.y / info_.resolution)
                + og_center_;
            cv::Point3i target = cv::Point3i(target_2d.x, target_2d.y, angle_to_index(t_yaw));
            
            RCLCPP_INFO(this->get_logger(),"Planning target: %.2f %.2f %.2f-> %d %d %d",
                    pose.pose.position.x, pose.pose.position.y, t_yaw, target.x, target.y, target.z);
            cv::circle(og_rgb_marked_,target_2d, 10, cv::Scalar(0,0,255));
            if (!headless_) {
                cv::imshow( "OccGrid", og_rgb_marked_ );
            }
            if (!isInGrid(target_2d)) {
                RCLCPP_ERROR(this->get_logger(),"Invalid target point (%.2f %.2f) -> (%d %d)",
                        pose.pose.position.x, pose.pose.position.y, target.x, target.y);
                return;
            }
            // Only accept target which are FREE in the grid (HW, Step 5).
            if (og_(target_2d) != FREE) {
                RCLCPP_ERROR(this->get_logger(),"Invalid target point: occupancy = %d",og_(target_2d));
                return;
            }

            // Now get the current point in grid coordinates.
            cv::Point2i start_2d;
            double s_yaw = 0;
            if (debug_) {
                start_2d = og_center_;
            } else {
                // For reference, this is how we get the current pose orientation
                s_yaw = tf2::getYaw(transformStamped.transform.rotation);
                start_2d = cv::Point2i(transformStamped.transform.translation.x / info_.resolution, 
                        transformStamped.transform.translation.y / info_.resolution)
                    + og_center_;
            }
            cv::Point3i start = cv::Point3i(start_2d.x, start_2d.y, angle_to_index(s_yaw));

            RCLCPP_INFO(this->get_logger(),"Planning origin %.2f %.2f %.2f -> %d %d %d",
                    transformStamped.transform.translation.x, transformStamped.transform.translation.y,
                    s_yaw, start.x, start.y, start.z);
            cv::circle(og_rgb_marked_,start_2d, 10, cv::Scalar(0,255,0));
            if (!headless_) {
                cv::imshow( "OccGrid", og_rgb_marked_ );
            }
            if (!isInGrid(start_2d)) {
                RCLCPP_ERROR(this->get_logger(),"Invalid starting point (%.2f %.2f) -> (%d %d)",
                        transformStamped.transform.translation.x, transformStamped.transform.translation.y,
                        start.x, start.y);
                return;
            }
            // If the starting point is not FREE there is a bug somewhere, but
            // better to check
            if (og_(start_2d) != FREE) {
                RCLCPP_ERROR(this->get_logger(),"Invalid start point: occupancy = %d",og_(start_2d));
                return;
            }
            RCLCPP_INFO(this->get_logger(),"Starting planning from (%d, %d) to (%d, %d)",start.x,start.y, target.x, target.y);

            planToPixelTarget(start, target);
        }
            

        void planToPixelTarget(cv::Point3i start, cv::Point3i target) {

            // Here the Dijskstra algorithm starts 
            // The best distance to the goal computed so far. This is
            // initialised with Not-A-Number. 
            int dims[3] = {og_.size().width, og_.size().height, 8};
            cv::Mat_<float> cell_value(3,dims, NAN);
            // For each cell we need to store a pointer to the coordinates of
            // its best predecessor. 
            cv::Mat_<cv::Vec3s> predecessor(3,dims);

            // Step 2
            Heap heap;
            heap.insert(Heap::value_type(0, start));
            cell_value(start.x,start.y,start.z) = 0;
            while (!heap.empty()) {
                // Select the cell at the top of the heap
                Heap::iterator hit = heap.begin();
                // the cell it contains is this_cell
                cv::Point3i this_cell = hit->second;

                // break if the target is already reached
                if (this_cell == target) {
                    break;
                }

                // and its score is this_cost
                float this_cost = cell_value(this_cell.x,this_cell.y,this_cell.z);
                // We can remove it from the heap now.
                heap.erase(hit);
                
                // Define neighbors 
                std::vector<std::pair<cv::Point3i, float>> moves;
                
                // 1. Move Forward
                double theta = index_to_angle(this_cell.z);
                
                int dx = 0, dy = 0;
                float move_cost = 1.0;
                
                // Map sector to dx, dy (Approximation)
                switch(this_cell.z) {
                    case 0: dx = 1; dy = 0; move_cost = 1.0; break;
                    case 1: dx = 1; dy = 1; move_cost = sqrt(2); break;
                    case 2: dx = 0; dy = 1; move_cost = 1.0; break;
                    case 3: dx = -1; dy = 1; move_cost = sqrt(2); break;
                    case 4: dx = -1; dy = 0; move_cost = 1.0; break;
                    case 5: dx = -1; dy = -1; move_cost = sqrt(2); break;
                    case 6: dx = 0; dy = -1; move_cost = 1.0; break;
                    case 7: dx = 1; dy = -1; move_cost = sqrt(2); break;
                }
                
                cv::Point3i forward_cell(this_cell.x + dx, this_cell.y + dy, this_cell.z);
                moves.push_back({forward_cell, move_cost});

                // 2. Turn Left (increase index)
                int left_z = (this_cell.z + 1) % 8;
                moves.push_back({cv::Point3i(this_cell.x, this_cell.y, left_z), 1.0}); // rotation cost

                // 3. Turn Right (decrease index)
                int right_z = (this_cell.z - 1 + 8) % 8;
                moves.push_back({cv::Point3i(this_cell.x, this_cell.y, right_z), 1.0}); // rotation cost


                // Now see where we can go from this_cell
                for (size_t i=0;i<moves.size();i++) {
                    cv::Point3i dest = moves[i].first;
                    float step_cost = moves[i].second;

                    if (!isInGrid(P2(dest))) {
                        // outside the grid
                        continue;
                    }
                    uint8_t og_val = og_(dest.y, dest.x); // Check 2D collision
                    if (og_val != FREE) {
                        // occupied or unknown
                        continue;
                    }
                    
                    float cv = cell_value(dest.x,dest.y,dest.z);
                    float new_cost = this_cost + step_cost;
                    
                    if (std::isnan(cv) || (new_cost < cv)) {
                        // found shortest path (or new path), updating the
                        // predecessor and the value of the cell
                        predecessor.at<cv::Vec3s>(dest.x,dest.y,dest.z) = cv::Vec3s(this_cell.x,this_cell.y,this_cell.z);
                        cell_value(dest.x,dest.y,dest.z) = new_cost;

                        //A*
                        float h = std::hypot(dest.x - target.x, dest.y - target.y); //heuristic (Euclidean distance on (x,y))
                        float priority = new_cost + h;
                        heap.insert(Heap::value_type(priority, dest));
                    }
                }
            }
            if (isnan(cell_value(target.x,target.y,target.z))) {
                // No path found
                RCLCPP_ERROR(this->get_logger(),"No path found from (%d, %d, %d) to (%d, %d, %d)",
                        start.x,start.y,start.z,target.x,target.y,target.z);

                cv::Point2i target_2d = cv::Point2i(target.x, target.y);
                auto it = std::find(frontier_points_vector.begin(), frontier_points_vector.end(), target_2d);
                if (it != frontier_points_vector.end()) {
                    frontier_points_vector.erase(it);
                }
                if (!frontier_points_vector.empty()) {
                    exploration_step(false);
                }
                return;
            }
            RCLCPP_INFO(this->get_logger(),"Planning completed");
            // Now extract the path by starting from goal and going through the
            // predecessors until the starting point
            std::list<cv::Point3i> lpath;
            while (target != start) {
                assert(lpath.size()<1000000);
                lpath.push_front(target);
                cv::Vec3s p = predecessor.at<cv::Vec3s>(target.x,target.y,target.z);
                target.x = p[0]; target.y = p[1]; target.z = p[2];
            }
            lpath.push_front(start);
            // Finally create a ROS path message
            nav_msgs::msg::Path path;
            path.header.stamp = this->get_clock()->now();
            path.header.frame_id = frame_id_;
            path.poses.resize(lpath.size());
            std::list<cv::Point3i>::const_iterator it = lpath.begin();
            unsigned int ipose = 0;
            while (it != lpath.end()) {
                // time stamp is not updated because we're not creating a
                // trajectory at this stage
                path.poses[ipose].header = path.header;
                cv::Point3i P = *it;
                cv::Point P2d = cv::Point(P.x, P.y) - og_center_; // Center offset
                path.poses[ipose].pose.position.x = (P2d.x) * info_.resolution;
                path.poses[ipose].pose.position.y = (P2d.y) * info_.resolution;
                
                tf2::Quaternion Q;
                Q.setRPY(0,0, index_to_angle(P.z));
                path.poses[ipose].pose.orientation = tf2::toMsg(Q);
                ipose++;
                it ++;
            }
            path_pub_->publish(path);
            RCLCPP_INFO(this->get_logger(),"Request completed");
        }



    public:
        OccupancyGridPlanner() : rclcpp::Node("occgrid_planner") {
            ready_ = false;
            this->declare_parameter("~/base_frame",std::string("body"));
            this->declare_parameter("~/debug",false);
            this->declare_parameter("~/headless",true);
            this->declare_parameter("~/robot_radius",0.1);
            base_link_ = this->get_parameter("~/base_frame").as_string();
            debug_ = this->get_parameter("~/debug").as_bool();
            headless_ = this->get_parameter("~/headless").as_bool();
            robot_radius_ = this->get_parameter("~/robot_radius").as_double();
            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
            og_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("~/occ_grid",1,
                    std::bind(&OccupancyGridPlanner::og_callback,this,std::placeholders::_1));
            target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("~/goal",1,
                    std::bind(&OccupancyGridPlanner::target_callback,this,std::placeholders::_1));
            path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path",1);
            optimal_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/optimal_target",1);
            
            explorer_service_ = this->create_service<std_srvs::srv::SetBool>(
                    "~/enable_explorer",
                    std::bind(&OccupancyGridPlanner::explorer_callback, 
                              this, std::placeholders::_1, std::placeholders::_2));

            if (!headless_) {
                cv::namedWindow( "OccGrid", cv::WINDOW_AUTOSIZE );
                timer_ = this->create_wall_timer( 50ms,
                        std::bind(&OccupancyGridPlanner::timer_cb, this));
            }

            // Timer to publish goal_pose every 10 seconds
            timer_exploration_ = this->create_wall_timer(
                    10s,
                    std::bind(&OccupancyGridPlanner::timer_exploration_callback, this));
        }

        void timer_cb() {
            cv::waitKey(5);
        }

        void timer_exploration_callback() {
            exploration_step();
        }

        void explorer_callback(
                const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            do_explore_ = request->data;
            response->success = true;
            response->message = do_explore_ ? "Exploration enabled" : "Exploration disabled";
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());

            if (!do_explore_) {
                // Publish an empty path so path_follower stops immediately
                nav_msgs::msg::Path empty_path;
                empty_path.header.stamp = this->get_clock()->now();
                empty_path.header.frame_id = frame_id_;
                path_pub_->publish(empty_path);
                frontier_points_vector.clear();
                has_current_target_ = false;
            }
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridPlanner>());
    rclcpp::shutdown();
}

