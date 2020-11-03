#ifndef SRC_SENSORDATARECEIVER_H
#define SRC_SENSORDATARECEIVER_H

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "map_sense/PlanarRegions.h"
#include "geometry_msgs/PoseStamped.h"

#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"

#include <iostream>
#include <CL/cl.hpp>
#include "math.h"
#include <sstream>
#include <random>

using namespace ros;
using namespace std;
using namespace chrono;
using namespace cv;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace map_sense;

class SensorDataReceiver {
public:
    ImageConstPtr sensorColorMessage;
    ImageConstPtr sensorDepthMessage;

    void depthCallback(const ImageConstPtr &depthMsg);
    void colorCallback(const sensor_msgs::ImageConstPtr &colorMsg);
    void get_sample_depth(Mat depth);
    void get_sample_color(Mat color);


};


#endif //SRC_SENSORDATARECEIVER_H
