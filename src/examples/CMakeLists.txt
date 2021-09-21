cmake_minimum_required(VERSION 3.4)
project(map_sense)

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