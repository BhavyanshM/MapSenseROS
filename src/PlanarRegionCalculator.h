#ifndef SRC_PLANARREGIONCALCULATOR_H
#define SRC_PLANARREGIONCALCULATOR_H

#define WIDTH 640
#define HEIGHT 480
#define GRID_X 8
#define GRID_Y 8

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "map_sense/PlanarRegions.h"
#include "geometry_msgs/PoseStamped.h"
#include "MapFrame.h"
#include "MapFrameProcessor.h"

#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"

#include <iostream>
#include <CL/cl.hpp>
#include "math.h"
#include <sstream>
#include <random>

#include "SensorDataReceiver.h"

using namespace ros;
using namespace std;
using namespace chrono;
using namespace cv;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace map_sense;

class PlanarRegionCalculator {
public:
    cl::Kernel filterKernel, packKernel, mergeKernel;
    cl::Context context;
    cl::CommandQueue commandQueue;
    cl::Image2D imgInBuf, imgOutBuf1, imgOutBuf2;
    cl::ImageFormat uint8_img, float_img;
    cl::size_t<3> origin, size;
    cl::Event event;

    ImageConstPtr colorMessage;
    ImageConstPtr depthMessage;

    const int SUB_H = 60;
    const int SUB_W = 80;
    Mat inputDepth = Mat(HEIGHT, WIDTH, CV_16UC1);
    Mat inputColor = Mat(HEIGHT, WIDTH, CV_8UC3);
    MapFrame output;
    MapFrameProcessor mapFrameProcessor;

    void fit();
    void init_opencl();
    void launch_tester();
};


#endif //SRC_PLANARREGIONCALCULATOR_H
