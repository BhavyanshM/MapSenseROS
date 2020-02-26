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
cl::Context context;
cl::CommandQueue queue;
cl::Buffer imgInBuf, imgOutBuf1, imgOutBuf2;
cl::ImageFormat uint8_img;
cl::ImageFormat float_img;
cl::size_t<3> origin;
cl::size_t<3> size;


void printMat(const Mat& image_cv){
		Vec4b point = image_cv.at<Vec4b>(cv::Point(384,512));
		cout << point << endl;
		printf("%d\t%d\t%d\n", image_cv.size().height, image_cv.size().width, image_cv.channels());
}

void fit(const Mat& image_cv){
    cl::Image2D imgInBuf(context, CL_MEM_READ_WRITE, uint8_img, WIDTH, HEIGHT, 0, &image_cv.data[0]);
	cl::Image2D imgOutBuf1(context, CL_MEM_READ_WRITE, float_img, SUB_W, SUB_H, 0, NULL);
    cl::Image2D imgOutBuf2(context, CL_MEM_READ_WRITE, float_img, SUB_W, SUB_H, 0, NULL);
 
    queue.enqueueNDRangeKernel(kern,cl::NullRange,cl::NDRange(10),cl::NullRange);
    queue.finish();

    // cout << "Fitting" << endl;


    queue.enqueueReadImage(out, CL_TRUE, origin, size, 0, 0, tmp);

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
    cl::Device default_device = all_devices[0];
    context = cl::Context({default_device});
   
    cl::Program::Sources sources;
    // calculates for each element; C = A + B

    FILE *fp;
	char *source_str;
	size_t source_size, program_size;

	fp = fopen("fitting_kernel.cl", "rb");
	if (!fp) {
	    printf("Failed to load kernel\n");
	    return 1;
	}

	fseek(fp, 0, SEEK_END);
	program_size = ftell(fp);
	rewind(fp);
	source_str = (char*)malloc(program_size + 1);
	source_str[program_size] = '\0';
	fread(source_str, sizeof(char), program_size, fp);
	cout << source_str << endl;
	fclose(fp);


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
 
    cl::ImageFormat uint8_img = cl::ImageFormat(CL_RGBA, CL_UNSIGNED_INT8);
	cl::ImageFormat float_img = cl::ImageFormat(CL_RGBA, CL_FLOAT); 
 
    queue = cl::CommandQueue(context,default_device); 
    kern = cl::Kernel(program,"fitting_kernel");
    kern.setArg(0,imgInBuf);
    kern.setArg(1,imgOutBuf1);
    kern.setArg(2,imgOutBuf2);

    origin[0] = 0;
	origin[1] = 0;
	origin[2] = 0;
	size[0] = WIDTH;
	size[1] = HEIGHT;
	size[2] = 4;

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