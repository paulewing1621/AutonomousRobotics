cmake_minimum_required(VERSION 3.8)
project(leica_pts_parser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -std=c++17 -ggdb)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mesh_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(tinyxml2 CONFIG REQUIRED)

find_package(VTK 9 REQUIRED)
include_directories(${PCL_COMMON_INCLUDE_DIRS})
include_directories(${VTK_INCLUDE_DIRS})

find_package(leica_pts_parser_msgs REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# set(msg_files "msg/ColoredMeshGeometryStamped.msg")
# set(srv_files "srv/GetPointCloud.srv" "srv/GetColoredGeometry.srv")
# rosidl_generate_interfaces(${PROJECT_NAME}
#       ${msg_files}
#       ${srv_files}
# )
# ament_export_dependencies(rosidl_default_runtime)

add_executable(leica_pts_parser_node src/leica_pts_parser_node.cpp)
target_include_directories(leica_pts_parser_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(leica_pts_parser_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(leica_pts_parser_node PUBLIC
    ${geometry_msgs_TARGETS}
    ${leica_pts_parser_msgs_TARGETS}
    ${mesh_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${shape_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    pcl_conversions::pcl_conversions
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs_library
    tf2_ros::static_transform_broadcaster_node
    tf2_ros::tf2_ros 
    ${PCL_LIBRARIES} tinyxml2
)


install(TARGETS leica_pts_parser_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
