#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_sense/PlanarRegion.h"
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
#define SUB_W 64
#define SUB_H 48


Publisher RGBDPosePub;
Publisher planarRegionPub;

cl::Kernel kern;
cl::CommandQueue queue;
cl::Buffer imgInBuf, imgOutBuf1, imgOutBuf2;


void printMat(const Mat& image_cv){
		Vec4b point = image_cv.at<Vec4b>(cv::Point(384,512));
		cout << point << endl;
		printf("%d\t%d\t%d\n", image_cv.size().height, image_cv.size().width, image_cv.channels());
}

void fit(const Mat& image_cv){
    queue.enqueueWriteBuffer(imgInBuf,CL_TRUE,0,sizeof(int)*10,A);
 
    queue.enqueueNDRangeKernel(kern,cl::NullRange,cl::NDRange(10),cl::NullRange);
    queue.finish();
 
    queue.enqueueWriteBuffer(imgOutBuf1,CL_TRUE,0,sizeof(int)*10,B);
    queue.enqueueReadBuffer(imgOutBuf2,CL_TRUE,0,sizeof(int)*10,C);
 
    cout<<" result: ";
    for(int i=0;i<10;i++){
        cout<<C[i]<<" ";
    }
    cout << endl;
}


void depthCallback(const CompressedImageConstPtr& msg){
	try{
		Mat image = imdecode(Mat(msg->data),1);
		Mat image_cv;
		cvtColor(image, image_cv, COLOR_RGB2RGBA);
		// circle(image, cv::Point(384,512), 12, cv::Scalar(0,0,255),1);	



		// PlanarRegion planeMsg;
		// planeMsg.header.stamp = Time::now();
		// planeMsg.header.frame_id = "/world_of_planes";
		// planeMsg.polygon.polygon.points.reserve(10);
		// for(int i = 0; i<10; i++){
		// 	Point32 point;
		// 	point.x = 2.0*i;
		// 	point.y = i;
		// 	point.z = 3*i;
		// 	planeMsg.polygon.polygon.points.push_back(point);
		// }

		// planarRegionPub.publish(planeMsg);

		fit(image_cv);

		// printMat(image_cv);


		imshow("DepthCallback", image);
		waitKey(1);

	}catch(cv_bridge::Exception& e){
		ROS_ERROR("Could not convert to image!");
	}
}



int main (int argc, char** argv){
	init(argc, argv, "PlanarRegionPublisher");

	NodeHandle nh;
	RGBDPosePub = nh.advertise<PoseStamped>("rgbd_pose", 1000);
	planarRegionPub = nh.advertise<PlanarRegion>("/map/regions/test", 1000);

	vector<cl::Platform> all_platforms;
	cl::Platform::get(&all_platforms);

    if (all_platforms.size()==0) {
        cout<<" No platforms found. Check OpenCL installation!\n";
        exit(1);
    }
    cl::Platform default_platform=all_platforms[0];
    vector<cl::Device> all_devices;
    default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
    cl::Device default_device=all_devices[0];
    cl::Context context({default_device});
   
    cl::Program::Sources sources;
    // calculates for each element; C = A + B
	std::string kernel_code=
            "   void kernel simple_add(global const int* A, global const int* B, global int* C){       "
            "       C[get_global_id(0)]=A[get_global_id(0)]+B[get_global_id(0)];                 "
            "   }                                                                               ";
    sources.push_back({kernel_code.c_str(),kernel_code.length()});
 
    cl::Program program(context,sources);
    if(program.build({default_device})!=CL_SUCCESS){
        cout<<" Error building: "<<program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(default_device)<<"\n";
        exit(1);
    }
 
    imgInBuf = cl::Image(context,CL_MEM_READ_WRITE,cl::ImageFormat(CL_RGBA, CL_UNSIGNED_INT8), WIDTH, HEIGHT, 0, (void*));
	imgOutBuf1 = cl::Image(context,CL_MEM_READ_WRITE,cl::ImageFormat(CL_RGBA, CL_FLOAT), SUB_W, SUB_H, 0, (void*));
	imgOutBuf2 = cl::Image(context,CL_MEM_READ_WRITE,cl::ImageFormat(CL_RGBA, CL_FLOAT), SUB_W, SUB_H, 0, (void*)); 
 
    queue = cl::CommandQueue(context,default_device); 
    kern = cl::Kernel(program,"simple_add");
    kern.setArg(0,imgInBuf);
    kern.setArg(1,imgOutBuf1);
    kern.setArg(2,imgOutBuf2);


	namedWindow("DepthCallback");
	Subscriber sub = nh.subscribe("/depth_image/compressed", 1, depthCallback);


	Rate loop_rate(200);
	int count = 0;
	int r = 3;

	while (ok()){
		spinOnce();
		loop_rate.sleep();
		++count;
	}
	destroyWindow("DepthCallback");
	return 0;
}