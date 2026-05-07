#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>


#include "leica_pts_parser/leica_point_type.h"
#include "leica_pts_parser/SDBParser.h"
#include "leica_pts_parser/XMLParser.h"
#include "leica_pts_parser_msgs/srv/get_point_cloud.hpp"
#include "leica_pts_parser_msgs/srv/get_colored_geometry.hpp"
#include "leica_pts_parser_msgs/msg/colored_mesh_geometry_stamped.hpp"
#include <mesh_msgs/msg/mesh_geometry_stamped.hpp>
#include <mesh_msgs/srv/get_geometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PTSLoader : public rclcpp::Node {
    protected:

        std::vector<pcl::PointCloud<PointXYZRGBI>> pc_;
        std::vector<pcl::PointCloud<PointXYZRGBI>> centered_pc_;
        std::vector<std::string> centered_pc_frame_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr agg_pub_;
        rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr meshPub_;
        rclcpp::Publisher<leica_pts_parser_msgs::msg::ColoredMeshGeometryStamped>::SharedPtr coloredmeshPub_;
        rclcpp::Publisher<shape_msgs::msg::Mesh>::SharedPtr shapePub_;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        // tf2_ros::StaticTransformBroadcaster static_broadcaster;
        std::string base_frame_;
        std::map<std::string,std::string> name_map_;
        std::string name_map_file_;
        bool publish_;
        bool publish_undecimated_;
        bool publish_point_tf_;
        bool uwb_anchors_;
        bool use_static_tf_;
        bool sdb_file_;
        bool pts_file_;
        bool xml_file_;
        bool rescale_intensity_;
        bool save_mesh_;
        double ind_publication_period_;
        int mesh_select_;
        double publication_period_;
        double edge_threshold_; // for vtk
        double vtk_decimate_; // for vtk
        std::string output_directory_;
        std::vector<std::string> output_file_names_;
        std::vector<geometry_msgs::msg::TransformStamped> point_tf_;
        std::string input_file_name_;
        leica_pts_parser_msgs::msg::ColoredMeshGeometryStamped m_full_mesh_;
        mesh_msgs::msg::MeshGeometryStamped m_mesh_;
        shape_msgs::msg::Mesh s_mesh_;
        size_t pub_index_;
        std::vector<sensor_msgs::msg::PointCloud2> rospc_;
        sensor_msgs::msg::PointCloud2 agg_rospc_;
        rclcpp::Time latest, ind_latest;
        bool ready_;

        std::string input_base(const std::string & extension) const {
            std::string base_name = input_file_name_;
            size_t pos = input_file_name_.rfind(extension);
            if (pos != std::string::npos) {
                base_name = input_file_name_.substr(0,pos);
            }
            pos = base_name.find_last_of("/");
            if (pos != std::string::npos) {
                base_name = base_name.substr(pos+1);
            }
            return base_name;
        }


#if 1
        rclcpp::Service<leica_pts_parser_msgs::srv::GetColoredGeometry>::SharedPtr coloredmeshSrv_;
        rclcpp::Service<mesh_msgs::srv::GetGeometry>::SharedPtr meshSrv_;
        rclcpp::Service<leica_pts_parser_msgs::srv::GetPointCloud>::SharedPtr pcSrv_;

        bool serveGeometry(const std::shared_ptr<mesh_msgs::srv::GetGeometry::Request> req,
                std::shared_ptr<mesh_msgs::srv::GetGeometry::Response> res)
        {
            RCLCPP_INFO(get_logger(),"Serving mesh geometry");
            res->mesh_geometry_stamped = m_mesh_;
            return true;
        }

        bool serveColoredGeometry(const std::shared_ptr<leica_pts_parser_msgs::srv::GetColoredGeometry::Request> req,
                std::shared_ptr<leica_pts_parser_msgs::srv::GetColoredGeometry::Response> res)
        {
            RCLCPP_INFO(get_logger(),"Serving colored mesh geometry (no decimation)");
            res->colored_mesh_geometry_stamped = m_full_mesh_;
            return true;
        }

        bool servePointCloud(const std::shared_ptr<leica_pts_parser_msgs::srv::GetPointCloud::Request> req,
                std::shared_ptr<leica_pts_parser_msgs::srv::GetPointCloud::Response> res)
        {
            RCLCPP_INFO(get_logger(),"Serving point cloud");
            res->cloud = agg_rospc_;
            return true;
        }
#endif


    public:
        bool pts_load() {
            pc_.clear();
            pc_.resize(1);
            centered_pc_.resize(1);
            centered_pc_frame_.resize(1);
            centered_pc_frame_[0] = base_frame_;
            output_file_names_.push_back(output_directory_+"/"+
                    input_base(".pts")+".pcd");
            FILE * fp=fopen(input_file_name_.c_str(),"r");
            if (!fp) {
                RCLCPP_ERROR(get_logger(),"Cannot open '%s' for reading",input_file_name_.c_str());
                return false;
            }
            int r;
            float imin=0,imax=0;
            unsigned int n;
            r = fscanf(fp," %u ", &n);
            if (r!=1) {
                return false;
            }
            while (!feof(fp)) {
                char line[1024];
                if (fgets(line,1024,fp)==NULL) {
                    break;
                }
                float x,y,z,intensity;
                int r,g,b;
                if (sscanf(line," %f %f %f %f %d %d %d ",&x,&y,&z,&intensity,&r,&g,&b)!=7) {
                    continue;
                }
                if (pc_[0].size()==0) {
                    imin = imax = intensity;
                } else {
                    imin = std::min<float>(intensity,imin);
                    imax = std::max<float>(intensity,imax);
                }
                pc_[0].push_back(PointXYZRGBI(x,y,z,r,g,b,intensity));
            }
            if (pc_[0].size() != n) {
                RCLCPP_ERROR(get_logger(),"PC size inconsistent with declared number of lines");
                return false;
            }
            centered_pc_[0] = pc_[0];
            if (rescale_intensity_) {
                for (size_t i=0;i<pc_[0].size();i++) {
                    pc_[0][i].intensity+=imin;
                }
                imax -= imin;
                imin = 0;
            }
            RCLCPP_INFO(get_logger(),"Loaded '%s': %d points (I in %f:%f)",input_file_name_.c_str(),int(pc_[0].size()),imin,imax);
            return true;
        }

        bool sdb_load() {
            pc_.clear();
            pc_.resize(1);
            centered_pc_frame_.resize(1);
            centered_pc_frame_[0] = base_frame_;
            std::string vtkOutput = output_directory_+"/"+ input_base(".sdb");
            output_file_names_.push_back(output_directory_+"/"+
                    input_base(".sdb")+".pcd");
            float imin=0,imax=0;
            LeicaSDBParser ldp;
            if (!ldp.load(input_file_name_)) {
                RCLCPP_ERROR(get_logger(),"Could not load %s with SDB parser",input_file_name_.c_str());
                return false;
            }
            for (size_t i=0;i<ldp.getPoints().size();i++) {
                const LeicaScanPoint & sp = ldp.getPoints()[i];
                if (pc_[0].size()==0) {
                    imin = imax = sp.intensity;
                } else {
                    imin = std::min<float>(sp.intensity,imin);
                    imax = std::max<float>(sp.intensity,imax);
                }
                pc_[0].push_back(PointXYZRGBI(sp.x,sp.y,sp.z,sp.r,sp.g,sp.b,sp.intensity,sp.snr));
            }
            if (rescale_intensity_) {
                for (size_t i=0;i<pc_[0].size();i++) {
                    pc_[0][i].intensity+=imin;
                }
                imax -= imin;
                imin = 0;
            }
            centered_pc_[0] = pc_[0];
            RCLCPP_INFO(get_logger(),"SDB Loaded '%s': %d points (I in %f:%f)",input_file_name_.c_str(),int(pc_[0].size()),imin,imax);
            if (save_mesh_) {
                RCLCPP_INFO(get_logger(),"Saving VTK mesh to %s",vtkOutput.c_str());
                ldp.exportMesh(edge_threshold_, 0.0,
                        &m_full_mesh_.mesh_geometry,&m_full_mesh_.vertex_color, NULL,false, 
                        vtkOutput+"_raw" , false, true, false);
                ldp.exportMesh(edge_threshold_, vtk_decimate_,
                        &m_mesh_.mesh_geometry, NULL, &s_mesh_,false, 
                        vtkOutput, true, true, false);
            }
            return true;
        }

        bool xml_load() {
            pc_.clear();
            centered_pc_.clear();
            centered_pc_frame_.clear();
            float imin=0,imax=0;
            LeicaXMLParser ldp;
            if (!ldp.load(input_file_name_)) {
                RCLCPP_ERROR(get_logger(),"Could not load %s with XML parser",input_file_name_.c_str());
                return false;
            }
            pc_.resize(ldp.numScans());
            centered_pc_.resize(ldp.numScans());
            centered_pc_frame_.resize(ldp.numScans());
            for (size_t j=0;j<ldp.numScans();j++) {
                RCLCPP_INFO(get_logger(),"Processing scan %d of %d (mesh select %d, save_mesh_ %d)",int(j),int(ldp.numScans()),int(mesh_select_),int(save_mesh_));
                pc_[j].clear();
                centered_pc_[j].clear();
                centered_pc_frame_[j] = ldp.getFrameName(j);
                if (centered_pc_frame_[j].empty()) {
                    centered_pc_frame_[j] = base_frame_;
                }
                std::string vtkOutput = output_directory_+"/"+ ldp.getScanName(j);
                output_file_names_.push_back(output_directory_+"/"+ ldp.getScanName(j)+".pcd");
                for (size_t i=0;i<ldp.getPoints(j).size();i++) {
                    const LeicaScanPoint & sp = ldp.getPoints(j)[i];
                    if ((j==0) && (i==0)) {
                        imin = imax = sp.intensity;
                    } else {
                        imin = std::min<float>(sp.intensity,imin);
                        imax = std::max<float>(sp.intensity,imax);
                    }
                    pc_[j].push_back(PointXYZRGBI(sp.x,sp.y,sp.z,sp.r,sp.g,sp.b,sp.intensity,sp.snr));
                }
                for (size_t i=0;i<ldp.getCenteredPoints(j).size();i++) {
                    const LeicaScanPoint & sp = ldp.getCenteredPoints(j)[i];
                    centered_pc_[j].push_back(PointXYZRGBI(sp.x,sp.y,sp.z,sp.r,sp.g,sp.b,sp.intensity,sp.snr));
                }
                if (save_mesh_ && ((mesh_select_<0)||(mesh_select_==int(j)))) {
                    bool save_this_mesh = ((mesh_select_<0) && (j==ldp.numScans()-1)) || ((mesh_select_>=0)&&(mesh_select_==int(j)));
                    RCLCPP_INFO(get_logger(),"Saving VTK mesh to %s",vtkOutput.c_str());
                    ldp.exportMesh(j,edge_threshold_, -1,
                            &m_full_mesh_.mesh_geometry,&m_full_mesh_.vertex_color, NULL,
                            (j>0), vtkOutput+"_raw" , false, save_this_mesh, false);
                    
                    ldp.exportMesh(j,edge_threshold_, vtk_decimate_,
                            &m_mesh_.mesh_geometry,NULL,&s_mesh_,(j>0), 
                            vtkOutput, save_this_mesh, save_this_mesh, false);

                    ldp.exportMesh(j,-1, -1, NULL,NULL,NULL,false, 
                            vtkOutput, false, true, true);
                }
            }
            if (rescale_intensity_) {
                for (size_t j=0;j<pc_.size();j++) {
                    for (size_t i=0;i<pc_[j].size();i++) {
                        pc_[j][i].intensity+=imin;
                    }
                }
                imax -= imin;
                imin = 0;
            }
            RCLCPP_INFO(get_logger(),"XML Loaded '%s': %d scans (I in %f:%f)",input_file_name_.c_str(),int(pc_.size()),imin,imax);
            rclcpp::Time rostime = this->get_clock()->now();


            if(uwb_anchors_){
                std::ofstream yamlFile(output_directory_+"/uwb_anchors_leica.yaml");
                std::string output_file_yaml=output_directory_+"/uwb_anchors_leica.yaml";
                if (yamlFile.is_open()) {

                    time_t now = time(0);
                    char *dt = ctime(&now);
                    std::cout << "The local date and time is: " << dt << std::endl;

                    yamlFile << "%YAML:1.0\n";
                    yamlFile << "%\n";
                    yamlFile << "% autogenerated by pts_loader with ROS time " << rostime.seconds() << "\n";
                    yamlFile << "% generated on " << dt << "\n\n";

                    const LeicaXMLParser::OffsetMap &surveyedPoints = ldp.getSurveyedPoints();
                    int i = 0;
                    std::set<std::string> yaml_anchors;
                    for (LeicaXMLParser::OffsetMap::const_iterator it = surveyedPoints.begin(); it != surveyedPoints.end(); it++,i++)
                    {
                        if (yaml_anchors.find(it->first) != yaml_anchors.end())
                        {
                            continue;
                        }
                        yaml_anchors.insert(it->first);
                        yamlFile << "anchor" << i << ":\n";
                        bool fix = true;
                        std::string leica_name = it->first;
                        if (!name_map_.empty()) {
                            std::map<std::string,std::string>::const_iterator nit = name_map_.find(it->first);
                            if (nit != name_map_.end()) {
                                leica_name = nit->second;
                            } else {
                                leica_name = it->first;
                            }
                        }
                        yamlFile << "  id: " << leica_name << "\n";
                        yamlFile << "  fix: " << fix << "\n";
                        yamlFile << "  p_AinG: [" << it->second.easting << ", " << it->second.northing << ", " << it->second.elevation << "]\n\n";
                    }
                    RCLCPP_INFO(get_logger(),"Saving YAML file to :  %s", output_file_yaml.c_str());
                    RCLCPP_INFO(get_logger(),"%d anchors has been saved ", i);
                }
                else
                {
                    std::cerr << "Failed to open yaml file" << std::endl;
                    return 1;
                }
            }


            if (publish_point_tf_) {
                const LeicaXMLParser::OffsetMap & surveyedPoints = ldp.getSurveyedPoints();
                for (LeicaXMLParser::OffsetMap::const_iterator it=surveyedPoints.begin(); it!=surveyedPoints.end(); it++) {
                    geometry_msgs::msg::TransformStamped static_transformStamped;
                    static_transformStamped.header.stamp = rostime;
                    static_transformStamped.header.frame_id = base_frame_;
                    if (name_map_.empty()) {
                        static_transformStamped.child_frame_id = it->first;
                    } else {
                        std::map<std::string,std::string>::const_iterator nit = name_map_.find(it->first);
                        if (nit != name_map_.end()) {
                            static_transformStamped.child_frame_id = nit->second;
                        } else {
                            static_transformStamped.child_frame_id = it->first;
                        }
                    }
                    static_transformStamped.transform.translation.x = it->second.easting;
                    static_transformStamped.transform.translation.y = it->second.northing;
                    static_transformStamped.transform.translation.z = it->second.elevation;
                    static_transformStamped.transform.rotation.x = 0;
                    static_transformStamped.transform.rotation.y = 0;
                    static_transformStamped.transform.rotation.z = 0;
                    static_transformStamped.transform.rotation.w = 1;

                    if (use_static_tf_) {
                        static_broadcaster_->sendTransform(static_transformStamped);
                    } else {
                        point_tf_.push_back(static_transformStamped);
                    }
                }

            }


            return true;
        }

        PTSLoader() : rclcpp::Node("leica_pts_parser") {
            this->declare_parameter("~/base_frame",std::string("/world"));
            this->declare_parameter("~/input_file_name",std::string("input"));
            this->declare_parameter("~/output_directory",std::string("output"));
            this->declare_parameter("~/name_map",name_map_file_);
            this->declare_parameter("~/vtk_edge_threshold",0.1);
            this->declare_parameter("~/vtk_decimate",0.9);
            this->declare_parameter("~/ind_publication_period",1.0);
            this->declare_parameter("~/publication_period",1.0);
            this->declare_parameter("~/save_mesh",true);
            this->declare_parameter("~/mesh_select",-1);
            this->declare_parameter("~/publish",false);
            this->declare_parameter("~/publish_undecimated",true);
            this->declare_parameter("~/publish_point_tf",true);
            this->declare_parameter("~/uwb_anchors_",true);
            this->declare_parameter("~/use_static_tf",true);
            this->declare_parameter("~/rescale_intensity",true);

            base_frame_ = this->get_parameter("~/base_frame").as_string();
            input_file_name_ = this->get_parameter("~/input_file_name").as_string();
            output_directory_ = this->get_parameter("~/output_directory").as_string();
            name_map_file_ = this->get_parameter("~/name_map").as_string();
            edge_threshold_ = this->get_parameter("~/vtk_edge_threshold").as_double();
            vtk_decimate_ = this->get_parameter("~/vtk_decimate").as_double();
            ind_publication_period_ = this->get_parameter("~/ind_publication_period").as_double();
            publication_period_ = this->get_parameter("~/publication_period").as_double();
            save_mesh_ = this->get_parameter("~/save_mesh").as_bool();
            mesh_select_ = this->get_parameter("~/mesh_select").as_int();
            publish_ = this->get_parameter("~/publish").as_bool();
            publish_undecimated_ = this->get_parameter("~/publish_undecimated").as_bool();
            publish_point_tf_ = this->get_parameter("~/publish_point_tf").as_bool();
            uwb_anchors_ = this->get_parameter("~/uwb_anchors_").as_bool();
            use_static_tf_ = this->get_parameter("~/use_static_tf").as_bool();
            rescale_intensity_ = this->get_parameter("~/rescale_intensity").as_bool();

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

            if (!name_map_file_.empty()) {
                FILE * fp = fopen(name_map_file_.c_str(),"r");
                if (fp) {
                    while (!feof(fp)) {
                        char line[1024];
                        char from[1024],to[1024];
                        if (fgets(line,1024,fp)==NULL) {
                            continue;
                        }
                        if (sscanf(line,"%s %s",from,to)!=2) {
                            continue;
                        }
                        name_map_[from] = to;
                    }
                    fclose(fp);
                }


            }


            ready_ = true;
            if (!input_file_name_.empty()) {
                pts_file_ = input_file_name_.rfind(".pts") == (input_file_name_.size()-4);
                sdb_file_ = input_file_name_.rfind(".sdb") == (input_file_name_.size()-4);
                xml_file_ = input_file_name_.rfind(".xml") == (input_file_name_.size()-4);
                if (pts_file_ && !pts_load()) {
                    ready_ = false;
                    throw std::runtime_error(std::string("Could not load pts file ") + input_file_name_);
                } else if (sdb_file_ && !sdb_load()) {
                    ready_ = false;
                    throw std::runtime_error(std::string("Could not load sdb file ") + input_file_name_);
                } else if (xml_file_ && !xml_load()) {
                    ready_ = false;
                    throw std::runtime_error(std::string("Could not load xml file ") + input_file_name_);
                } else {
                    RCLCPP_INFO(get_logger(),"Loaded %s",input_file_name_.c_str());
                    if (!output_directory_.empty()) {
                        RCLCPP_INFO(get_logger(),"Saving %d pointclouds",int(output_file_names_.size()));
                        for (size_t i=0;i<output_file_names_.size();i++) {
                            try {
                                pcl::io::savePCDFileBinary (output_file_names_[i], pc_[i]);
                                RCLCPP_INFO(get_logger(),"Saved point cloud to '%s'",output_file_names_[i].c_str());
                            } catch (pcl::IOException & e) {
                                RCLCPP_ERROR(get_logger(),"Cannot save point cloud to '%s'. Check if file exists",output_file_names_[i].c_str());
                            }
                        }
                    }
                }
            }

            rclcpp::Time now = this->get_clock()->now();
            latest = ind_latest = now;
            if (ready_ && publish_) {
                pcl::PointCloud<PointXYZRGBI> allpc;
                rospc_.clear();
                for (size_t i=0;i<pc_.size();i++) {
                    sensor_msgs::msg::PointCloud2 rpc;
                    pcl::PCLPointCloud2 cloud2;
                    pcl::toPCLPointCloud2(centered_pc_[i],cloud2);
                    pcl_conversions::fromPCL(cloud2,rpc);
                    pcl::toROSMsg(centered_pc_[i],rpc);
                    rpc.header.stamp = now;
                    rpc.header.frame_id = centered_pc_frame_[i];
                    rospc_.push_back(rpc);
                    if ((mesh_select_>=0) && (mesh_select_!=int(i))) {
                        continue;
                    }
                    allpc.insert(allpc.end(),pc_[i].begin(),pc_[i].end());
                }
                bool latched = ind_publication_period_ <= 0;
                bool agg_latched = publication_period_ <= 0;
                rclcpp::QoS latching_qos(1);
                rclcpp::QoS agg_latching_qos(1);
                if (latched) {
                    latching_qos = latching_qos.transient_local();
                }
                if (agg_latched) {
                    agg_latching_qos = agg_latching_qos.transient_local();
                }
                
                pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/cloud",latching_qos);
                pub_index_ = 0;
                agg_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/aggregated_cloud",agg_latching_qos);
                pcl::toROSMsg(allpc,agg_rospc_);
                agg_rospc_.header.stamp = now;
                agg_rospc_.header.frame_id = base_frame_;
                agg_pub_->publish(agg_rospc_);
                if (xml_file_ || sdb_file_) {
                    m_mesh_.header.stamp = agg_rospc_.header.stamp;
                    m_mesh_.header.frame_id = base_frame_;
                    m_mesh_.uuid = "UUID";
                    m_full_mesh_.header = m_mesh_.header;
                    m_full_mesh_.uuid = m_mesh_.uuid;

                    meshPub_ = this->create_publisher<mesh_msgs::msg::MeshGeometryStamped>("~/mesh",latching_qos); /* latched */
                    meshPub_->publish(m_mesh_);
                    if (publish_undecimated_) {
                        coloredmeshPub_ = this->create_publisher<leica_pts_parser_msgs::msg::ColoredMeshGeometryStamped>("colored_mesh",latching_qos); /* latched */
                        coloredmeshPub_->publish(m_full_mesh_);
                    }

                    shapePub_ = this->create_publisher<shape_msgs::msg::Mesh>("~/shape",latching_qos); /* latched */
                    shapePub_->publish(s_mesh_);

                }

                meshSrv_ = this->create_service<mesh_msgs::srv::GetGeometry>("~/get_mesh", 
                        std::bind(&PTSLoader::serveGeometry,this,std::placeholders::_1,std::placeholders::_2));
                coloredmeshSrv_ = this->create_service<leica_pts_parser_msgs::srv::GetColoredGeometry>("~/get_colored_mesh", 
                        std::bind(&PTSLoader::serveColoredGeometry,this,std::placeholders::_1,std::placeholders::_2));
                pcSrv_ = this->create_service<leica_pts_parser_msgs::srv::GetPointCloud>("~/get_pc", 
                        std::bind(&PTSLoader::servePointCloud,this,std::placeholders::_1,std::placeholders::_2));

                timer_ = this->create_wall_timer( std::chrono::duration<double>(0.1),
                        std::bind(&PTSLoader::timer_cb, this));

                RCLCPP_INFO(this->get_logger(), "spinning...");
            }
        }

        void timer_cb() {
            rclcpp::Time now = this->get_clock()->now();
            if (ready_ && publish_ && (rospc_.size()>0) && (ind_publication_period_>0) && ((now-ind_latest).seconds() > ind_publication_period_)) {
                if (pub_index_>=rospc_.size()) {
                    pub_index_ = 0;
                }
                rospc_[pub_index_].header.stamp = ind_latest = now;
                // RCLCPP_INFO(get_logger(),"Publishing scan %d",int(pub_index_));
                pub_->publish(rospc_[pub_index_]);
                pub_index_+=1;
            }
            if (ready_ && publish_ && (publication_period_>0) && ((now-latest).seconds() > publication_period_)) {
                agg_rospc_.header.stamp = latest = now;
                m_full_mesh_.header.stamp = m_mesh_.header.stamp = agg_rospc_.header.stamp;
                agg_pub_->publish(agg_rospc_);
                meshPub_->publish(m_mesh_);
                if (publish_undecimated_) {
                    coloredmeshPub_->publish(m_full_mesh_);
                }
                shapePub_->publish(s_mesh_);
            }
            if (publish_point_tf_ && !use_static_tf_) {
                for (size_t i=0;i<point_tf_.size();i++) {
                    point_tf_[i].header.stamp = now;
                    tf_broadcaster_->sendTransform(point_tf_[i]);
                }
            }
        }
};

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PTSLoader>());
    rclcpp::shutdown();
    return 0;
}

