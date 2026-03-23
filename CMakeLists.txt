cmake_minimum_required(VERSION 3.8)
project(filter_reflective_uavs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mrs_lib REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ouster_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseVelocityArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_library(filter_reflective_uavs_component SHARED
  src/filter_reflective_uavs.cpp
)
add_dependencies(filter_reflective_uavs_component ${PROJECT_NAME}__rosidl_generator_cpp)

target_include_directories(filter_reflective_uavs_component
  PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(filter_reflective_uavs_component
  geometry_msgs
  mrs_lib
  mrs_msgs
  nav_msgs
  ouster_ros
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

target_link_libraries(filter_reflective_uavs_component
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(filter_reflective_uavs_component "${cpp_typesupport_target}")

rclcpp_components_register_nodes(filter_reflective_uavs_component
  "filter_reflective_uavs::FilterReflectiveUavs"
)

install(
  TARGETS filter_reflective_uavs_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
