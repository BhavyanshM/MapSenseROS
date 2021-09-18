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
		src/SLAM/SLAMModule.cpp
		src/ImPlot/implot.cpp
		src/ImPlot/implot_demo.cpp
		src/ImPlot/implot_items.cpp
        src/FeatureDetection/PlanarRegionProcessor.cpp
		src/FileManager.cpp
		src/FeatureDetection/KeypointDetector.cpp
		src/OpenCL/OpenCLManager.cpp
		src/Rendering/MeshGenerator.cpp
		)

set (HEADERS
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
		src/headers/SLAMModule.h
		src/headers/PlanarRegionMapHandler.h
		src/headers/FactorGraphHandler.h
        src/FeatureDetection/PlanarRegionProcessor.h
		src/headers/FileManager.h
		src/headers/KeypointDetector.h
		src/headers/OpenCLManager.h
		src/headers/Log.h
		src/headers/Instrumentor.h
		src/headers/Timer.h src/headers/Core.h)

set(MAPSENSE_GUI_SOURCE
		src/Launcher/MapsenseLauncherUI.cpp
		src/UI/ImGuiLayout.cpp
		)


set(MAPSENSE_GUI_HEADERS
		src/headers/MeshGenerator.h
		src/headers/MapsenseLauncherUI.h
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
		src/headers/Timer.h src/headers/Core.h)

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
		src/Geometry/src/Modules/StructureFromMotion.cpp src/headers/MapsenseHeaders.h src/headers/Log.h)

include_directories(
		include
		src/headers
		src/Geometry/include
		${catkin_INCLUDE_DIRS}
		/usr/include/CL/
		Vendor/spdlog/include
)

add_executable(Mapsense_Launcher_UI ${SOURCE} ${MAPSENSE_GUI_SOURCE} ${HEADERS} ${MAPSENSE_GUI_HEADERS} )
target_precompile_headers(Mapsense_Launcher_UI PUBLIC src/headers/MapsenseHeaders.h)
target_link_libraries(Mapsense_Launcher_UI
		${OpenCV_LIBS}
		${catkin_LIBRARIES}
		${MAGNUM_GUI_LIBS}
		MagnumIntegration::ImGui
		/usr/lib/x86_64-linux-gnu/libOpenCL.so
		Eigen3::Eigen
		tbb
		gtsam
		geom-tools
		${TORCH_LIBRARIES}
		ClayEngine)
add_dependencies(Mapsense_Launcher_UI map_sense_generate_messages_cpp)
set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
#set_target_properties(Mapsense_Launcher_UI PROPERTIES LINK_FLAGS -Wl,-z, stack-size, 1000000000000)


add_executable(TorchDemo src/examples/TorchDemo.cpp)
target_link_libraries(TorchDemo PRIVATE
		${TORCH_LIBRARIES})

add_executable(factorGraph2D src/examples/FactorGraph2D.cpp)
target_link_libraries(factorGraph2D PRIVATE
		${MAGNUM_GUI_LIBS})

add_library(geom-tools STATIC ${GEOMTOOLS_SOURCE} ${GEOMTOOLS_HEADERS})
target_link_libraries(geom-tools
		Eigen3::Eigen
		${OpenCV_LIBS})

add_executable(registration_and_mapping
		src/Application/AppUtils.cpp
		src/Rendering/MeshGenerator.cpp
        src/Geometry/src/Elements/PlanarRegion.cpp
		src/SLAM/PlanarRegionSimulator.cpp
		src/Launcher/MagnumApplication.cpp
		src/Launcher/SLAMApplication.cpp
		src/Application/PlanarRegionMapTester.cpp
		src/Geometry/src/GeomTools.cpp
		src/SLAM/PlanarRegionMapHandler.cpp
		src/SLAM/FactorGraphHandler.cpp
		src/headers/AppUtils.h
		src/headers/SLAMApplication.h
		src/headers/PlanarRegion.h
		src/headers/PlanarRegionSimulator.h
		src/headers/MagnumApplication.h
		src/headers/PlanarRegionMapTester.h
		src/headers/MeshGenerator.h
		src/headers/PlanarRegionMapHandler.h
		src/headers/FactorGraphHandler.h
		src/headers/Instrumentor.h
		src/headers/Timer.h src/headers/Core.h)
target_link_libraries(registration_and_mapping
		Eigen3::Eigen
		${OpenCV_LIBS}
		${MAGNUM_GUI_LIBS}
		${catkin_LIBRARIES}
		gtsam
		geom-tools)
include_directories(${GTEST_INCLUDE_DIRS})
target_link_libraries(registration_and_mapping ${GTEST_LIBRARIES})


add_executable(mapsense_headless
        ${SOURCE} ${HEADERS}
		src/Launcher/MapsenseHeadlessLauncher.cpp
		src/headers/MapsenseHeadlessLauncher.h)
target_link_libraries(mapsense_headless
		${OpenCV_LIBS}
		${catkin_LIBRARIES}
		${MAGNUM_GUI_LIBS}
		MagnumIntegration::ImGui
		/usr/lib/x86_64-linux-gnu/libOpenCL.so
		Eigen3::Eigen
		tbb
		gtsam
		geom-tools)
add_dependencies(mapsense_headless map_sense_generate_messages_cpp)

add_executable(planar-slam src/examples/PlanarSLAMExample.cpp)
target_link_libraries(planar-slam gtsam)

add_executable(pose2-slam src/examples/Pose2SLAMExample.cpp)
target_link_libraries(pose2-slam gtsam)

add_executable(visual-isam2 src/examples/VisualISAM2Example.cpp src/examples/SFMdata.h)
target_link_libraries(visual-isam2 gtsam)

add_executable(math-tester
		src/examples/MathTester.cpp)

target_link_libraries(math-tester
		Eigen3::Eigen)






