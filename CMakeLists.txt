cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_to_rangeimage)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  dynamic_reconfigure
  message_generation
  image_transport
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)


generate_dynamic_reconfigure_options(
    cfg/RangeImage.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS PCL
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(range_image_spherical
#   src/${PROJECT_NAME}/range_image_spherical.cpp
# )

## Declare a cpp executable
add_executable(pointcloud_to_rangeimage_node src/pointcloud_to_rangeimage_node.cpp)
add_dependencies(pointcloud_to_rangeimage_node ${PROJECT_NAME}_gencfg pointcloud_to_rangeimage_generate_messages_cpp)
target_link_libraries(pointcloud_to_rangeimage_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)

## Declare a cpp executable
add_executable(rangeimage_to_pointcloud_node src/rangeimage_to_pointcloud_node.cpp)
add_dependencies(rangeimage_to_pointcloud_node ${PROJECT_NAME}_gencfg pointcloud_to_rangeimage_generate_messages_cpp)
target_link_libraries(rangeimage_to_pointcloud_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)

#############
## Install ##
#############

