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

add_message_files(
	FILES
	RawGPUPlanarRegionList.msg
	RawGPUPlanarRegion.msg
	MapsenseConfiguration.msg
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

set(SOURCE
		src/RegionExtraction/PlanarRegionCalculator.cpp
		src/Network/NetworkManager.cpp
		src/RegionExtraction/MapFrame.cpp
		src/RegionExtraction/MapFrameProcessor.cpp
		src/RegionExtraction/PlanarRegion.cpp
		src/Application/ApplicationState.cpp
		src/RegionExtraction/RegionRing.cpp
		src/SLAM/PlanarRegionMapHandler.cpp
		src/Application/AppUtils.cpp
		src/Geometry/GeomTools.cpp
		)

set (HEADERS
		src/headers/PlanarRegionCalculator.h
		src/headers/NetworkManager.h
		src/headers/MapFrame.h
		src/headers/MapFrameProcessor.h
		src/headers/PlanarRegion.h
		src/headers/ApplicationState.h
		src/headers/RegionRing.h
		src/headers/PlanarRegionMapHandler.h
		src/headers/AppUtils.h
		src/headers/GeomTools.h)

set(MAPSENSE_GUI_SOURCE
		src/Rendering/MeshGenerator.cpp
		src/Launcher/ScenePrimitives.cpp
		src/UI/ImGuiLayout.cpp)


set(MAPSENSE_GUI_HEADERS
		src/headers/MeshGenerator.h
		src/headers/ScenePrimitives.h
		src/headers/ImGuiLayout.h )

set(MAGNUM_GUI_LIBS
		Corrade::Main
		Magnum::Application
		Magnum::GL
		Magnum::Magnum
		Magnum::MeshTools
		Magnum::Primitives
		Magnum::SceneGraph
		Magnum::Shaders
		Magnum::Trade)

include_directories(
		include
		src/headers
		${catkin_INCLUDE_DIRS}
		/usr/include/CL/
)

add_executable(planar_region_publisher ${SOURCE} ${MAPSENSE_GUI_SOURCE} ${HEADERS} ${MAPSENSE_GUI_HEADERS})
target_link_libraries(planar_region_publisher
		${OpenCV_LIBS}
		${catkin_LIBRARIES}
		${MAGNUM_GUI_LIBS}
		MagnumIntegration::ImGui
		/usr/lib/x86_64-linux-gnu/libOpenCL.so
		Eigen3::Eigen
		tbb)
add_dependencies(planar_region_publisher map_sense_generate_messages_cpp)
set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
#set_target_properties(planar_region_publisher PROPERTIES LINK_FLAGS -Wl,-z, stack-size, 1000000000000)



add_executable(factorGraph2D src/examples/FactorGraph2D.cpp)
target_link_libraries(factorGraph2D PRIVATE
		${MAGNUM_GUI_LIBS})

add_executable(registration_and_mapping
		src/SLAM/PlanarRegionMapHandler.cpp
		src/RegionExtraction/PlanarRegion.cpp
		src/SLAM/PlanarRegionSimulator.cpp
		src/Launcher/MagnumApplication.cpp
		src/Launcher/SLAMApplication.cpp
		src/headers/SLAMApplication.h
		src/headers/PlanarRegionMapHandler.h
		src/headers/PlanarRegion.h
		src/headers/PlanarRegionSimulator.h
		src/headers/MagnumApplication.h
		src/Application/PlanarRegionMapTester.cpp
		src/Application/PlanarRegionMapTester.h
		src/headers/GeomTools.h
		src/Geometry/GeomTools.cpp)
target_link_libraries(registration_and_mapping
		Eigen3::Eigen
		${MAGNUM_GUI_LIBS})


add_executable(mapsense_headless
        ${SOURCE} ${HEADERS}
		src/Launcher/MapsenseHeadlessLauncher.cpp src/headers/MapsenseHeadlessLauncher.h)
target_link_libraries(mapsense_headless
		${OpenCV_LIBS}
		${catkin_LIBRARIES}
		/usr/lib/x86_64-linux-gnu/libOpenCL.so
		Eigen3::Eigen
		tbb)
add_dependencies(mapsense_headless map_sense_generate_messages_cpp)






