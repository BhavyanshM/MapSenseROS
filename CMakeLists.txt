cmake_minimum_required(VERSION 3.4)
project(map_sense)

# This line is required for specifying the "modules" directory in the project containing .cmake files
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/../../devel;/opt/ros/noetic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#MATH(EXPR stack_size "2048000 * 1024 * 1024") # 16 Mb
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=${stack_size}")

enable_testing()
find_package(GTest REQUIRED) # Find the google testing framework on your system
find_package(Torch REQUIRED PATHS "${PROJECT_SOURCE_DIR}/Vendor/libtorch")

find_package(VTK 7 REQUIRED)
find_package(gflags REQUIRED)
find_package( OpenCV 4.2 REQUIRED )
find_package(OpenCL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTSAM REQUIRED)

include_directories(${GTSAM_INCLUDE_DIR})

#find_package(Corrade REQUIRED Main)
#find_package(Magnum REQUIRED
#		MeshTools
#		GL
#		Primitives
#		Shaders
#		Sdl2Application
#		SceneGraph)
#
#find_package(MagnumIntegration REQUIRED ImGui)
#find_package(ImGui REQUIRED)


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
  pcl_ros
  pcl_conversions
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
        src/FeatureDetection/PlanarRegionCalculator.cpp
		src/Network/NetworkManager.cpp
        src/FeatureDetection/MapFrame.cpp
        src/FeatureDetection/MapFrameProcessor.cpp
        src/Geometry/src/Elements/PlanarRegion.cpp
		src/Application/ApplicationState.cpp
		src/Geometry/src/Elements/RegionRing.cpp
		src/Application/AppUtils.cpp
		src/ImageProcessing/ImageTools.cpp
		src/Network/ROS1TopicReceiver.cpp
        src/Network/ImageReceiver.cpp
		src/SLAM/PlanarRegionMapHandler.cpp
		src/SLAM/FactorGraphHandler.cpp
		src/ImPlot/implot.cpp
		src/ImPlot/implot_demo.cpp
		src/ImPlot/implot_items.cpp
		src/FeatureDetection/PlanarRegionProcessor.cpp
		src/FileManager.cpp
		src/FeatureDetection/KeypointDetector.cpp
		src/OpenCL/OpenCLManager.cpp
		src/SLAM/SLAMModule.cpp
		src/UI/ImGuiLayout.cpp
		src/SLAM/IterativeClosestPoint.cpp)

set (HEADERS
		src/headers/SLAMModule.h
		src/headers/PlanarRegionCalculator.h
		src/headers/NetworkManager.h
		src/headers/MapFrame.h
		src/headers/MapFrameProcessor.h
		src/headers/PlanarRegion.h
		src/headers/ApplicationState.h
		src/headers/RegionRing.h
		src/headers/AppUtils.h
		src/headers/ImageTools.h
		src/headers/ImageReceiver.h
		src/headers/ROS1TopicReceiver.h
		src/headers/PlanarRegionMapHandler.h
		src/headers/FactorGraphHandler.h
		src/FeatureDetection/PlanarRegionProcessor.h
		src/headers/FileManager.h
		src/headers/KeypointDetector.h
		src/headers/OpenCLManager.h
		src/headers/Log.h
		src/headers/Instrumentor.h
		src/headers/Timer.h src/headers/Core.h
		src/headers/ImGuiLayout.h
		src/headers/MapsenseHeaders.h
		src/headers/IterativeClosestPoint.h)

set(MAPSENSE_CLAY_HEADERS
		src/headers/ClayApp.h
		src/headers/ClayApp.h
		src/headers/PointCloudReceiver.h
		src/headers/MapsenseLayer.h)
set(MAPSENSE_CLAY_SOURCES
		src/Launcher/ClayApp.cpp
		src/Launcher/ClayApp.cpp
		src/Launcher/MapsenseLayer.cpp
		src/Network/PointCloudReceiver.cpp)

set(GEOMTOOLS_HEADERS
		src/Geometry/include/GeomTools.h
		src/Geometry/include/KDTree.h
		src/Geometry/include/KDNode.h
		src/Geometry/include/RigidBodyTransform.h
		src/Geometry/include/LieAlgebra.h
		src/Geometry/include/Rotation.h
		src/Geometry/include/Line3D.h
		src/Geometry/include/Plane3D.h
		src/Geometry/include/Point3D.h
		src/Geometry/include/StructureFromMotion.h
		src/headers/Instrumentor.h
		src/headers/Timer.h
		src/headers/Core.h)

set(GEOMTOOLS_SOURCE
		src/Geometry/src/GeomTools.cpp
		src/Geometry/src/Structures/KDTree.cpp
		src/Geometry/src/Structures/KDNode.cpp
		src/Geometry/src/Elements/RigidBodyTransform.cpp
		src/Geometry/src/Elements/Rotation.cpp
		src/Geometry/src/Elements/LieAlgebra.cpp
		src/Geometry/src/Elements/Point3D.cpp
		src/Geometry/src/Elements/Line3D.cpp
		src/Geometry/src/Elements/Plane3D.cpp
		src/Geometry/src/Modules/StructureFromMotion.cpp
		src/headers/Log.h)

include_directories(
		include
		src/headers
		src/Geometry/include
		${catkin_INCLUDE_DIRS}
		/usr/include/CL/
		Vendor/spdlog/include
)

#set_target_properties(Mapsense_Launcher_UI PROPERTIES LINK_FLAGS -Wl,-z, stack-size, 1000000000000)

add_executable(Mapsense_Clay ${SOURCE} ${MAPSENSE_CLAY_SOURCES} ${HEADERS} ${MAPSENSE_CLAY_HEADERS} )
target_include_directories(Mapsense_Clay PUBLIC
		"/usr/local/include/ClayEngine"
		"/usr/local/include/ClayEngine/Vendor/GLAD/include"
		"/usr/local/include/ClayEngine/Vendor/glfw/include"
		"/usr/local/include/ClayEngine/Vendor/glm"
		"/usr/local/include/ClayEngine/Vendor/ImGui"
		"/usr/local/include/ClayEngine/Vendor/ImGui/imgui"
		"/usr/local/include/ClayEngine/Vendor/spdlog/include"
		"/usr/local/include/ClayEngine/Vendor/stb_image/")
target_link_libraries(Mapsense_Clay
		${OpenCV_LIBS}
		${catkin_LIBRARIES}
		/usr/lib/x86_64-linux-gnu/libOpenCL.so
		Eigen3::Eigen
		tbb
		gtsam
		geom-tools
		${TORCH_LIBRARIES}
		ClayEngine
		glad imgui glfw3 ${CMAKE_DL_LIBS} )
add_dependencies(Mapsense_Clay map_sense_generate_messages_cpp)
target_precompile_headers(Mapsense_Clay PUBLIC src/headers/MapsenseHeaders.h)
target_compile_definitions(Mapsense_Clay PUBLIC -DASSETS_PATH="/usr/local/include/ClayEngine/Assets/")


add_library(geom-tools STATIC ${GEOMTOOLS_SOURCE} ${GEOMTOOLS_HEADERS})
target_link_libraries(geom-tools
		Eigen3::Eigen
		${OpenCV_LIBS})

if(BUILD_HEADLESS)
	add_executable(mapsense_headless
			${SOURCE} ${HEADERS}
			src/Launcher/MapsenseHeadlessLauncher.cpp
			src/headers/MapsenseHeadlessLauncher.h)
	target_include_directories(mapsense_headless PUBLIC
			"/usr/local/include/ClayEngine"
			"/usr/local/include/ClayEngine/Vendor/GLAD/include"
			"/usr/local/include/ClayEngine/Vendor/glfw/include"
			"/usr/local/include/ClayEngine/Vendor/glm"
			"/usr/local/include/ClayEngine/Vendor/ImGui"
			"/usr/local/include/ClayEngine/Vendor/ImGui/imgui"
			"/usr/local/include/ClayEngine/Vendor/spdlog/include"
			"/usr/local/include/ClayEngine/Vendor/stb_image/")
	target_link_libraries(mapsense_headless
			${OpenCV_LIBS}
			${catkin_LIBRARIES}
			/usr/lib/x86_64-linux-gnu/libOpenCL.so
			Eigen3::Eigen
			tbb
			gtsam
			geom-tools
			${TORCH_LIBRARIES})
	add_dependencies(mapsense_headless map_sense_generate_messages_cpp)
endif()
#		glad imgui glfw3 ${CMAKE_DL_LIBS} )

if(BUILD_EXAMPLES)
	add_subdirectory(src/examples/)
endif()






