cmake_minimum_required(VERSION 2.8.3)
project(map_sense)

find_package(VTK 7 REQUIRED)
find_package(gflags REQUIRED)
find_package( OpenCV 4.2 REQUIRED )
find_package(OpenCL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
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
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)


add_message_files(
	FILES
	PlanarRegions.msg
)

generate_messages(
	DEPENDENCIES
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
  /usr/include/CL/
)

add_executable(planar_region_publisher src/PlanarRegionPublisher.cpp)
target_link_libraries(planar_region_publisher
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libOpenCL.so
)

#add_dependencies(planar_region_publisher map_sense_generate_messages_cpp)





