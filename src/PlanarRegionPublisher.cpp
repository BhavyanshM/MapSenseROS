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

using namespace ros;
using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace map_sense;

#define WIDTH 1024
#define HEIGHT 786
#define SUB_W 16
#define SUB_H 12


Publisher RGBDPosePub;
Publisher planarRegionPub;

cl::Kernel kernel;
cl::Context context;
cl::CommandQueue queue;
cl::Image2D imgInBuf, imgOutBuf1, imgOutBuf2;
cl::ImageFormat uint8_img, float_img;
cl::size_t<3> origin, size;
cl::Event event;

ImageConstPtr colorMessage;
ImageConstPtr depthMessage;

void get_sample_depth(Mat mat);

void get_sample_color(Mat mat);

void fit(const Mat& color, const Mat& depth){

    float *prevDepthBuffer = reinterpret_cast<float *>(depth.data);
    cl::Image2D clDepth(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, cl::ImageFormat(CL_R, CL_FLOAT), WIDTH, HEIGHT , 0, prevDepthBuffer);
    cl::Image2D clOutput(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), WIDTH, HEIGHT);
    Mat output;

    kernel.setArg(0, clDepth);
    kernel.setArg(1, clOutput);

    queue.enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(SUB_H, SUB_W), cl::NullRange);
    // queue.enqueueReadImage(clDepth, CL_TRUE, origin, jacSize, 0, 0, output.data);
    queue.finish();

    // queue.enqueueReadImage(out, CL_TRUE, origin, size, 0, 0, tmp);
}

void depthCallback(const ImageConstPtr& depthMsg){
    depthMessage = depthMsg;
}

void colorCallback(const sensor_msgs::ImageConstPtr& colorMsg){
    colorMessage = colorMsg;
}

void processDataCallback(const TimerEvent&){
    cv_bridge::CvImagePtr img_ptr_depth;
    cv_bridge::CvImagePtr img_ptr_color;
    if(colorMessage != nullptr && depthMessage != nullptr){
        try {
            // ROS_INFO("Callback: Color:%d Depth:%d", colorMessage->header.stamp.sec, depthMessage->header.stamp.sec);

            img_ptr_color = cv_bridge::toCvCopy(*colorMessage, image_encodings::TYPE_8UC3);
            Mat colorImg = img_ptr_color->image;

            img_ptr_depth = cv_bridge::toCvCopy(*depthMessage, image_encodings::TYPE_16UC1);
            Mat depthImg = img_ptr_depth->image;
            depthImg.convertTo(depthImg, -1, 10, 100);

            fit(colorImg, depthImg);

            imshow("RealSense L515 Depth", depthImg);
            imshow("RealSense L515 Color", colorImg);
            int code = waitKeyEx(32);
            if (code == 1048689) exit(1);

        } catch(cv_bridge::Exception& e){
            ROS_ERROR("Could not convert to image!");
        }
    }
}

int main (int argc, char** argv){

    // export ROSCONSOLE_FORMAT='${time:format string}'
    // export ROSCONSOLE_FORMAT='[${severity}] [${time}][${node}:${line}]: ${message}'

    origin[0] = 0;
	origin[1] = 0;
	origin[2] = 0;
	size[0] = WIDTH;
	size[1] = HEIGHT;
	size[2] = 1;

	init(argc, argv, "PlanarRegionPublisher");
	NodeHandle nh;
	RGBDPosePub = nh.advertise<PoseStamped>("rgbd_pose", 1000);
	planarRegionPub = nh.advertise<PlanarRegions>("/map/regions/test", 1000);

	vector<cl::Platform> all_platforms;
	cl::Platform::get(&all_platforms);

    if (all_platforms.size()==0) {
        cout<<" No platforms found. Check OpenCL installation!\n";
        exit(1);
    }
    cl::Platform default_platform=all_platforms[0];
    vector<cl::Device> all_devices;
    default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
    cl::Device default_device = all_devices[0];
    context = cl::Context({default_device});
   
    cl::Program::Sources sources;
    // calculates for each element; C = A + B

    FILE *fp;
	char *source_str;
	size_t source_size, program_size;

	fp = fopen((ros::package::getPath("map_sense") + "/kernels/fitting_kernel.cpp").c_str(), "rb");
	if (!fp) {
	    printf("Failed to load kernel\n");
	    cout << ros::package::getPath("map_sense") + "/kernels/fitting_kernel.cpp" << endl;
	    return 1;
	}

	fseek(fp, 0, SEEK_END);
	program_size = ftell(fp);
	rewind(fp);
	source_str = (char*)malloc(program_size + 1);
	source_str[program_size] = '\0';
	fread(source_str, sizeof(char), program_size, fp);
	fclose(fp);

	std::string kernel_code(source_str);
    sources.push_back({kernel_code.c_str(),kernel_code.length()});
    cl::Program program(context,sources);
    if(program.build({default_device})!=CL_SUCCESS){
        cout<<" Error building: "<<program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(default_device)<<"\n";
        exit(1);
    }

    queue = cl::CommandQueue(context,default_device);
    kernel = cl::Kernel(program, "segmentKernel");

    Mat inputDepth(HEIGHT, WIDTH, CV_32FC1);
    Mat inputColor(HEIGHT, WIDTH, CV_8UC3);
    get_sample_depth(inputDepth);
    get_sample_color(inputColor);
    fit()


	// Subscriber subDepth = nh.subscribe("/camera/depth/image_rect_raw", 3, depthCallback);
    // Subscriber subColor = nh.subscribe("/camera/color/image_raw", 3, colorCallback);
    // Timer timer1 = nh.createTimer(Duration(0.1), processDataCallback);
	// spin();

	return 0;
}

void get_sample_color(Mat color) {

}

void get_sample_depth(Mat depth) {

}
