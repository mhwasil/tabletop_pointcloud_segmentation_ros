cmake_minimum_required(VERSION 2.8.3)
project(tabletop_pointcloud_segmentation_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  pcl_ros
  perception_msgs
  message_filters
  cv_bridge
  image_geometry
)

#find_package(Boost REQUIRED
#  message_filters
#)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/PlaneSegmentation.cfg
)

catkin_package(
  INCLUDE_DIRS
    ros/include
  LIBRARIES
    plane_segmentation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ros/include
  common/include
)

add_definitions(-fpermissive)

### LIBRARIES ####################################################
#add_library(bounding_box
#  common/src/bounding_box.cpp
#)

#add_dependencies(bounding_box
#  ${PROJECT_NAME}_gencpp
#  ${catkin_EXPORTED_TARGETS}
#)
#target_link_libraries(bounding_box
#  ${PCL_LIBRARIES}
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#)

# plane segmentation
#add_library(plane_segmentation
#  common/src/bounding_box.cpp
#  src/plane_segmentation.cpp
#)

#add_dependencies(plane_segmentation
#  ${PROJECT_NAME}_gencpp
#  ${catkin_EXPORTED_TARGETS}
#)
#target_link_libraries(plane_segmentation
#  ${PCL_LIBRARIES}
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#)


### EXECUTABLES ####################################################

add_executable(table_top_segmentation 
    common/src/bounding_box.cpp
    ros/src/plane_segmentation.cc
    ros/src/table_top_segmentation_ros.cpp
)
add_dependencies(table_top_segmentation
   ${catkin_EXPORTED_TARGETS}
   ${PROJECT_NAME}_gencfg
)
target_link_libraries(table_top_segmentation 
    ${catkin_LIBRARIES}
    ${Boost_INCLUDE_DIRS}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

### INSTALLS
install(DIRECTORY common/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
