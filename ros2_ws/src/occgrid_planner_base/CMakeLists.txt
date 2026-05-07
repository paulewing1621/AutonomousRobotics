cmake_minimum_required(VERSION 3.8)
project(occgrid_planner_base)
cmake_policy(SET CMP0074 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -g -ggdb -O3)
endif()

# find dependencies
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cs7630_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV includes: '${OpenCV_INCLUDE_DIRS}'")

add_executable(occgrid_planner_base src/occgrid_planner.cpp)
target_include_directories(occgrid_planner_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(occgrid_planner_base PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(occgrid_planner_base PUBLIC
    ${cs7630_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs_library
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::static_transform_broadcaster_node
    tf2_ros::tf2_ros
    ${OpenCV_LIBRARIES}
)

# add_executable(occgrid_planner_2d src/occgrid_planner_2D_copy.cpp)
# target_include_directories(occgrid_planner_2d PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(occgrid_planner_2d PUBLIC c_std_99 cxx_std_17)
# target_link_libraries(occgrid_planner_2d PUBLIC
#     ${cs7630_msgs_TARGETS}
#     ${geometry_msgs_TARGETS}
#     ${nav_msgs_TARGETS}
#     ${sensor_msgs_TARGETS}
#     ${std_msgs_TARGETS}
#     rclcpp::rclcpp
#     sensor_msgs::sensor_msgs_library
#     tf2_geometry_msgs::tf2_geometry_msgs
#     tf2_ros::static_transform_broadcaster_node
#     tf2_ros::tf2_ros
#     ${OpenCV_LIBRARIES}
# )

add_executable(path_optimizer_base src/path_optimizer.cpp)
target_include_directories(path_optimizer_base PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(path_optimizer_base PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(path_optimizer_base PUBLIC
    ${cs7630_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs_library
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::static_transform_broadcaster_node
    tf2_ros::tf2_ros
)

add_executable(path_follower_base src/path_follower.cpp)
target_include_directories(path_follower_base PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(path_follower_base PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(path_follower_base PUBLIC
    ${cs7630_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs_library
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::static_transform_broadcaster_node
    tf2_ros::tf2_ros
)

install(TARGETS occgrid_planner_base path_optimizer_base path_follower_base
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)


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
