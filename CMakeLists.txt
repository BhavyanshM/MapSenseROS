cmake_minimum_required(VERSION 3.4)
project(map_sense)

# This line is required for specifying the "modules" directory in the project containing .cmake files
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/../../devel;/opt/ros/noetic")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#MATH(EXPR stack_size "2048000 * 1024 * 1024") # 16 Mb
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=${stack_size}")

enable_testing()
find_package(GTest REQUIRED) # Find the google testing framework on your system
#find_package(Torch REQUIRED PATHS "${PROJECT_SOURCE_DIR}/Vendor/libtorch")

find_package(VTK 7 REQUIRED)
find_package(gflags REQUIRED)
find_package( OpenCV REQUIRED )
find_package(OpenCL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTSAM REQUIRED)
find_package(xtl REQUIRED)

add_subdirectory(Vendor/xtensor)

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

set(PLANAR_REGION_LIB
		src/FeatureDetection/PlanarRegionCalculator.cpp
		src/FeatureDetection/MapFrame.cpp
		src/FeatureDetection/MapFrameProcessor.cpp
		src/Geometry/src/Elements/PlanarRegion.cpp
		src/Geometry/src/Elements/RegionRing.cpp
		src/FeatureDetection/PlanarRegionProcessor.cpp
		src/OpenCL/OpenCLManager.cpp
		src/Application/AppUtils.cpp

		src/headers/AppUtils.h
		src/headers/OpenCLManager.h
		src/headers/PlanarRegionCalculator.h
		src/headers/MapFrameProcessor.h
		src/headers/MapFrame.h
		src/headers/PlanarRegion.h
		src/headers/RegionRing.h
		src/FeatureDetection/PlanarRegionProcessor.h)

set(SLAM_LIB
		src/SLAM/PlanarRegionMapHandler.cpp
		src/SLAM/FactorGraphHandler.cpp
		src/SLAM/SLAMModule.cpp
		src/headers/SLAMModule.h
		src/headers/FactorGraphHandler.h
		src/headers/PlanarRegionMapHandler.h)


set(ICP_LIB
		src/OpenCL/OpenCLManager.cpp
		src/headers/OpenCLManager.h
		src/SLAM/IterativeClosestPoint.cpp
		src/headers/IterativeClosestPoint.h)

set(SOURCE
		src/Application/ApplicationState.cpp
		src/Network/NetworkManager.cpp
		src/Application/AppUtils.cpp
		src/ImageProcessing/ImageTools.cpp
		src/Network/ROS1TopicReceiver.cpp
		src/Network/ImageReceiver.cpp
		src/ImPlot/implot.cpp
		src/ImPlot/implot_demo.cpp
		src/ImPlot/implot_items.cpp
		src/DataManager.cpp
		src/SLAM/VisualOdometry.cpp
		src/OpenCL/OpenCLManager.cpp
		src/UI/ImGuiLayout.cpp
		)

set (HEADERS


		src/headers/NetworkManager.h
		src/headers/ApplicationState.h
		src/headers/AppUtils.h
		src/headers/ImageTools.h
		src/headers/ImageReceiver.h
		src/headers/ROS1TopicReceiver.h
		src/headers/DataManager.h
		src/headers/VisualOdometry.h
		src/headers/OpenCLManager.h
		src/headers/Log.h
		src/headers/Instrumentor.h
		src/headers/Timer.h src/headers/Core.h
		src/headers/ImGuiLayout.h
		src/headers/MapsenseHeaders.h
		)

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
		src/Geometry/include/NonLinearOptimizer.h
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
		src/Geometry/include/PointLandmark.h)

set(GEOMTOOLS_SOURCE
		src/Geometry/src/Optimizers/NonLinearOptimizer.cpp
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
		src/Geometry/src/Elements/PointLandmark.cpp)

include_directories(
		include
		src/headers
		src/Geometry/include
		${catkin_INCLUDE_DIRS}
		/usr/include/CL/
		Vendor/spdlog/include
		Vendor/xtensor/include
		/usr/local/include/ClayEngine/Vendor/
)

#set_target_properties(Mapsense_Launcher_UI PROPERTIES LINK_FLAGS -Wl,-z, stack-size, 1000000000000)

#[[
	Develop all libraries here.
]]
add_library(geom-tools STATIC ${GEOMTOOLS_SOURCE} ${GEOMTOOLS_HEADERS})
target_link_libraries(geom-tools
		Eigen3::Eigen
		${OpenCV_LIBS})

add_library(planar-regions STATIC ${PLANAR_REGION_LIB})
target_link_libraries(planar-regions
		Eigen3::Eigen
		${OpenCV_LIBS}
		geom-tools)

add_library(slam STATIC ${SLAM_LIB})
target_link_libraries(slam
		gtsam
		Eigen3::Eigen
		${OpenCV_LIBS}
		geom-tools)

add_library(icp STATIC ${ICP_LIB})
target_link_libraries(icp Eigen3::Eigen)

#[[
	Develop all executables here.
]]
add_executable(Mapsense_Clay ${SOURCE} ${MAPSENSE_CLAY_SOURCES} ${HEADERS} ${MAPSENSE_CLAY_HEADERS} )
target_include_directories(Mapsense_Clay PUBLIC
		"/usr/local/include/ClayEngine"
		"/usr/local/include/ClayEngine/Vendor/"
		"/usr/local/include/ClayEngine/Vendor/GLAD/include"
		"/usr/local/include/ClayEngine/Vendor/glfw/include"
		"/usr/local/include/ClayEngine/Vendor/glm"
		"/usr/local/include/ClayEngine/Vendor/ImGui"
		"/usr/local/include/ClayEngine/Vendor/ImGui/imgui"
		"/usr/local/include/ClayEngine/Vendor/spdlog/include"
		"/usr/local/include/ClayEngine/Vendor/stb_image/")
target_link_libraries(Mapsense_Clay
		${OpenCV_LIBS} ${catkin_LIBRARIES}
		/usr/lib/x86_64-linux-gnu/libOpenCL.so
		Eigen3::Eigen tbb gtsam
		geom-tools planar-regions slam icp
		ClayEngine
		glad imgui glfw3 ${CMAKE_DL_LIBS} )
add_dependencies(Mapsense_Clay map_sense_generate_messages_cpp)
target_precompile_headers(Mapsense_Clay PUBLIC src/headers/MapsenseHeaders.h)
target_compile_definitions(Mapsense_Clay PUBLIC -DASSETS_PATH="/usr/local/include/ClayEngine/Assets/")



if(BUILD_HEADLESS)
	add_executable(mapsense_headless
			${SOURCE} ${HEADERS}
			src/Launcher/MapsenseHeadlessLauncher.cpp
			src/headers/MapsenseHeadlessLauncher.h)
	target_include_directories(mapsense_headless PUBLIC
			"/usr/local/include/ClayEngine/Vendor/"
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
			)
	add_dependencies(mapsense_headless map_sense_generate_messages_cpp)
endif()
#		glad imgui glfw3 ${CMAKE_DL_LIBS} )

if(BUILD_EXAMPLES)
	add_subdirectory(src/examples/)
endif()






