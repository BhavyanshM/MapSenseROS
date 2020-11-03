cmake_minimum_required(VERSION 3.4)
project(map_sense)

# This line is required for specifying the "modules" directory in the project containing .cmake files
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(VTK 7 REQUIRED)
find_package(gflags REQUIRED)
find_package( OpenCV 4.2 REQUIRED )
find_package(OpenCL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Corrade REQUIRED Main)
find_package(Magnum REQUIRED
		MeshTools
		GL
		Primitives
		Shaders
		Sdl2Application
		SceneGraph)

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

set(SOURCE_FILES
		src/PlanarRegionPublisher.cpp
		src/RegionFinder.cpp
		src/GPU.cpp
		src/PlanarRegionCalculator.cpp
		src/SensorDataReceiver.cpp
		src/ScenePrimitives.cpp)

set (HEADERS
		include/RegionFinder.h
		include/GPU.h
		src/PlanarRegionCalculator.h
		src/SensorDataReceiver.h)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/CL/
)

add_executable(planar_region_publisher ${SOURCE_FILES} ${HEADERS})
target_link_libraries(planar_region_publisher
	tbb
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
	/usr/lib/x86_64-linux-gnu/libOpenCL.so
	Corrade::Main
	Magnum::Application
	Magnum::GL
	Magnum::Magnum
	Magnum::MeshTools
	Magnum::Primitives
	Magnum::Shaders
	Magnum::SceneGraph
)

#set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")


set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

#add_executable(magnum-primitives WIN32 src/ScenePrimitives.cpp)

#target_include_directories(magnum-primitives PUBLIC "../eigen-3.3.7")


#add_dependencies(planar_region_publisher map_sense_generate_messages_cpp)





