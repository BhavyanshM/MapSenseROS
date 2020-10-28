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

cl::Kernel kern;
cl::Context context;
cl::CommandQueue queue;
cl::Image2D imgInBuf, imgOutBuf1, imgOutBuf2;
cl::ImageFormat uint8_img, float_img;
cl::size_t<3> origin, size;

cl::Event event;

void printMat(const Mat& image_cv){
		Vec4b point = image_cv.at<Vec4b>(cv::Point(384,512));
		cout << point << endl;
		printf("%d\t%d\t%d\n", image_cv.size().height, image_cv.size().width, image_cv.channels());
}

void fit(const Mat& image_cv){
    imgInBuf = cl::Image2D(context, CL_MEM_READ_ONLY, uint8_img, WIDTH, HEIGHT, 0, NULL);
    int e0 = kern.setArg(0,imgInBuf);
    cout << "E0:"<< e0 << endl;
    

    unsigned char *input = (unsigned char*)(image_cv.data);

    cl_uchar4* img = (cl_uchar4*)malloc(HEIGHT*WIDTH*4*8*4);
    cl::Event writeEvt;
    int status = queue.enqueueWriteImage(imgInBuf,CL_FALSE,origin,size,0,0,img);
    cout << "EnqueueWrite:" << status << endl;
    // printf("%d\t%d\t%d\n", image_cv.size().height, image_cv.size().width, image_cv.channels());

    queue.flush();

 	cl::Event temp;
    
    // cout << "Fitting" << endl;
    int err = queue.enqueueNDRangeKernel(kern, cl::NullRange, cl::NDRange(SUB_H, SUB_W), cl::NDRange(1,1), NULL, &temp);
    cout << "EnqueueNDRangeKernel:" << err << endl;


    event = temp;
    event.wait();
    queue.flush();
    queue.finish();



    // queue.enqueueReadImage(out, CL_TRUE, origin, size, 0, 0, tmp);

}


void depthCallback(const ImageConstPtr& msg){
    cv_bridge::CvImagePtr img_ptr_depth;

    try{
        img_ptr_depth = cv_bridge::toCvCopy(*msg, image_encodings::TYPE_16UC1);

        Mat img = img_ptr_depth->image;
        img.convertTo(img, -1, 10, 100);


        imshow("DepthCallback", img);
        int code = waitKeyEx(32);
        cout << "Code: " << code << endl;
        if (code == 1048689) exit(1);


		// cvtColor(image, image_cv, COLOR_RGB2RGBA);
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

		// fit(image_cv);

		// printMat(image_cv);


		// imshow("DepthCallback", image);
		// waitKey(1);

	}catch(cv_bridge::Exception& e){
		ROS_ERROR("Could not convert to image!");
	}
}



int main (int argc, char** argv){
    origin[0] = 0;
	origin[1] = 0;
	origin[2] = 0;
	size[0] = WIDTH;
	size[1] = HEIGHT;
	size[2] = 1;

    uint8_img = cl::ImageFormat(CL_RGBA, CL_UNSIGNED_INT8);
	float_img = cl::ImageFormat(CL_RGBA, CL_FLOAT); 

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

	fp = fopen((ros::package::getPath("map_sense") + "/scripts/fitting_kernel.cl").c_str(), "rb");
	if (!fp) {
	    printf("Failed to load kernel\n");
	    cout << ros::package::getPath("map_sense") + "/scripts/fitting_kernel.cl" << endl;
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
    kern = cl::Kernel(program,"segmentKernel");

	imgOutBuf1 = cl::Image2D(context, CL_MEM_READ_WRITE, float_img, SUB_W, SUB_H, 0, NULL);
    imgOutBuf2 = cl::Image2D(context, CL_MEM_READ_WRITE, float_img, SUB_W, SUB_H, 0, NULL);
    int e1 = kern.setArg(1,imgOutBuf1);
    int e2 = kern.setArg(2,imgOutBuf2);
    // cout <<  e1 << e2 << endl;

	namedWindow("DepthCallback");
	Subscriber sub = nh.subscribe("/camera/depth/image_rect_raw", 1, depthCallback);


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