#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_sense/PlanarRegion.h"
#include "geometry_msgs/PoseStamped.h"

#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"


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


Publisher RGBDPosePub;
Publisher planarRegionPub;


void printMat(const Mat& image_cv){
		Vec4b point = image_cv.at<Vec4b>(cv::Point(384,512));
		cout << point << endl;
		printf("%d\t%d\t%d\n", image_cv.size().height, image_cv.size().width, image_cv.channels());
}

void fit(const Mat& image_cv){
	printMat(image_cv);
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

		printMat(image_cv);
		imshow("DepthCallback", image);
		waitKey(1);

	}catch(cv_bridge::Exception& e){
		ROS_ERROR("Could not convert to image!");
	}
}

int main (int argc, char** argv){
	init(argc, argv, "PlanarRegionPublisher");

	NodeHandle n;
	RGBDPosePub = n.advertise<PoseStamped>("rgbd_pose", 1000);
	planarRegionPub = n.advertise<PlanarRegion>("/map/regions/test", 1000);

	namedWindow("DepthCallback");
	Subscriber sub = n.subscribe("/depth_image/compressed", 1, depthCallback);


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