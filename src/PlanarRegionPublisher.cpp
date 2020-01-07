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

#define DEPTH_WIDTH 1024
#define DEPTH_HEIGHT 786


Publisher RGBDPosePub;
Publisher planarRegionPub;

void depthCallback(const CompressedImageConstPtr& msg){
	try{
		Mat image = imdecode(Mat(msg->data),1);
		circle(image, cv::Point(384,512), 12, cv::Scalar(0,0,255),1);	
		Point3f point = image.at<Point3f>(cv::Point(384,512));
		printf("%d\t%d\t%d\n", image.size().height, image.size().width, image.depth());

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
	planarRegionPub = n.advertise<PlanarRegion>("planar_regions", 1000);

	namedWindow("DepthCallback");
	Subscriber sub = n.subscribe("/depth_image/compressed", 1, depthCallback);


	// image_transport::ImageTransport it(n);
	// image_transport::TransportHints hints("compressed");
	// image_transport::Subscriber sub = it.subscribe("/depth_image", 1, depthCallback, VoidPtr(), hints);





	Rate loop_rate(200);

	int count = 0;
	int r = 3;

	while (ok()){


		PoseStamped msg;
		msg.header.stamp = Time::now();
		msg.header.frame_id = "/world";
		msg.pose.position.x = r*cos(count*0.01);
		msg.pose.position.y = r*sin(count*0.01);
		msg.pose.position.z = 3.0;
		msg.pose.orientation.x = 0.0;
		msg.pose.orientation.y = 0.3826834;
		msg.pose.orientation.z = 0.0;
		msg.pose.orientation.w = 0.9238795;
		RGBDPosePub.publish(msg);


		spinOnce();

		loop_rate.sleep();
		++count;


	}
	destroyWindow("DepthCallback");

	return 0;
}