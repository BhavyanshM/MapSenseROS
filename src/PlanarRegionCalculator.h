#ifndef SRC_PLANARREGIONCALCULATOR_H
#define SRC_PLANARREGIONCALCULATOR_H

#define WIDTH 640
#define HEIGHT 480
#define GRID_X 8
#define GRID_Y 8

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "MapFrame.h"
#include "MapFrameProcessor.h"
#include "PlanarRegion.h"

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
#include "PlanarRegion.h"

#include "map_sense/RawGPUPlanarRegionList.h"

using namespace ros;
using namespace std;
using namespace chrono;
using namespace cv;




class PlanarRegionCalculator {
public:
    cl::Kernel filterKernel, packKernel, mergeKernel;
    cl::Context context;
    cl::CommandQueue commandQueue;
    cl::Image2D imgInBuf, imgOutBuf1, imgOutBuf2;
    cl::ImageFormat uint8_img, float_img;
    cl::size_t<3> origin, size;
    cl::Event event;

    Publisher planarRegionPub;

    SensorDataReceiver* _dataReceiver;

    const int SUB_H = 60;
    const int SUB_W = 80;
    Mat inputDepth = Mat(HEIGHT, WIDTH, CV_16UC1);
    Mat inputColor = Mat(HEIGHT, WIDTH, CV_8UC3);
    Mat filteredDepth = Mat(HEIGHT, WIDTH, CV_16UC1);

    MapFrame output;
    MapFrameProcessor mapFrameProcessor;
    vector<shared_ptr<PlanarRegion>> planarRegionList;

    void generatePatchGraph(ApplicationState appState);
    void init_opencl();
    void launch_tester(ApplicationState appState);
    void generate_regions(SensorDataReceiver* receiver, ApplicationState appState);
    void getFilteredDepth(Mat& dispDepth, bool showGraph);

};


#endif //SRC_PLANARREGIONCALCULATOR_H
