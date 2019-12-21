cmake_minimum_required(VERSION 2.8.3)
project(map_sense)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)


add_message_files(
	FILES
	PlanarRegion.msg
)

generate_messages(
	DEPENDENCIES
	geometry_msgs
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(planar_region_publisher src/PlanarRegionPublisher.cpp)
target_link_libraries(planar_region_publisher ${catkin_LIBRARIES})
add_dependencies(planar_region_publisher map_sense_generate_messages_cpp)





