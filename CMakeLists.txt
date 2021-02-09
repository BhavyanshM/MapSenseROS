cmake_minimum_required(VERSION 3.4)
project(map_sense)

# This line is required for specifying the "modules" directory in the project containing .cmake files
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

#MATH(EXPR stack_size "2048000 * 1024 * 1024") # 16 Mb
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=${stack_size}")

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

find_package(MagnumIntegration REQUIRED ImGui)
#find_package(ImGui REQUIRED)

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

#find_package(OpenCV REQUIRED)


add_message_files(
	FILES
	RawGPUPlanarRegionList.msg
	RawGPUPlanarRegion.msg
	MapSenseParams.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
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
		src/PlanarRegionCalculator.cpp
		src/NetworkManager.cpp
		src/ScenePrimitives.cpp
		src/MapFrame.cpp
		src/MapFrameProcessor.cpp
		src/PlanarRegion.cpp
		src/MeshGenerator.cpp
		src/ApplicationState.cpp
		src/RegionRing.cpp
		src/PlanarRegionMapHandler.cpp)

set (HEADERS
		src/headers/PlanarRegionCalculator.h
		src/headers/NetworkManager.h
		src/headers/MapFrame.h
		src/headers/MapFrameProcessor.h
		src/headers/PlanarRegion.h
		src/headers/MeshGenerator.h
		src/headers/ApplicationState.h
		src/headers/ScenePrimitives.h
		src/headers/RegionRing.h
		src/headers/PlanarRegionMapHandler.h)

include_directories(
  include
  src/headers
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
	MagnumIntegration::ImGui
	Eigen3::Eigen
)

#set_target_properties(planar_region_publisher PROPERTIES LINK_FLAGS -Wl,-z, stack-size, 1000000000000)

add_dependencies(planar_region_publisher map_sense_generate_messages_cpp)
set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

add_executable(factorGraph2D src/examples/FactorGraph2D.cpp)
target_link_libraries(factorGraph2D PRIVATE
		Corrade::Main
		Magnum::Application
		Magnum::GL
		Magnum::Magnum
		Magnum::MeshTools
		Magnum::Primitives
		Magnum::SceneGraph
		Magnum::Shaders
		Magnum::Trade)





