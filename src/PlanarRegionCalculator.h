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

#include "NetworkManager.h"
#include "PlanarRegion.h"

#include "tbb/tbb.h"

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

    NetworkManager* _dataReceiver;

    const int SUB_H = 60;
    const int SUB_W = 80;
    Mat inputDepth = Mat(HEIGHT, WIDTH, CV_16UC1);
    Mat inputColor = Mat(HEIGHT, WIDTH, CV_8UC3);
    Mat filteredDepth = Mat(HEIGHT, WIDTH, CV_16UC1);

    MapFrame output;
    MapFrameProcessor mapFrameProcessor;
    vector<shared_ptr<PlanarRegion>> planarRegionList, currentRegionList, previousRegionList;
    vector<int> matchIndices;

    void generatePatchGraph(ApplicationState appState);
    void initOpenCL();
    void launch_tester(ApplicationState appState);
    void generateRegions(NetworkManager* receiver, ApplicationState appState);
    void publishRegions(vector<shared_ptr<PlanarRegion>> regionList);
    void getFilteredDepth(Mat& dispDepth, bool showGraph);
    void getInputDepth(Mat& dispDepth, bool showGraph);
    void registerRegions(vector<shared_ptr<PlanarRegion>> prevRegions, vector<shared_ptr<PlanarRegion>> curRegions, vector<int>& matchIndices);
    static void onMouse(int event, int x, int y, int flags, void *userdata);
};


#endif //SRC_PLANARREGIONCALCULATOR_H
