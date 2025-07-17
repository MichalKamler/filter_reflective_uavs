cmake_minimum_required(VERSION 3.0.2)
project(filter_reflective_uavs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  mrs_lib
  mrs_msgs
  nav_msgs
  nodelet
  ouster_ros
  pcl_conversions
  pcl_ros
  roscpp
  tf2
  tf2_ros
)

find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES filter_reflective_uavs
 CATKIN_DEPENDS  mrs_lib  mrs_msgs  nav_msgs  nodelet  ouster_ros  pcl_conversions  pcl_ros  roscpp  tf2  tf2_ros
#  DEPENDS system_lib
)


# install(FILES plugin.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_library(FilterReflectiveUavs src/filter_reflective_uavs.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(FilterReflectiveUavs
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_include_directories(FilterReflectiveUavs SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})

install(TARGETS FilterReflectiveUavs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)