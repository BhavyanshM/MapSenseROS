#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_sense/PlanarRegion.h"
#include "geometry_msgs/PoseStamped.h"

#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"


#include "math.h"
#include <sstream>

ros::Publisher RGBDPosePub;
ros::Publisher planarRegionPub;

void depthCallback(const sensor_msgs::CompressedImageConstPtr& msg){
	try{
		cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);

		// map_sense::PlanarRegion planeMsg;
		// planeMsg.header.stamp = ros::Time::now();
		// planeMsg.header.frame_id = "/world_of_planes";
		// planeMsg.polygon.polygon.points.reserve(10);
		// for(int i = 0; i<10; i++){
		// 	geometry_msgs::Point32 point;
		// 	point.x = 2.0*i;
		// 	point.y = i;
		// 	point.z = 3*i;
		// 	planeMsg.polygon.polygon.points.push_back(point);
		// }

		// planarRegionPub.publish(planeMsg);

		cv::imshow("DepthCallback", image);
		cv::waitKey(1);

	}catch(cv_bridge::Exception& e){
		ROS_ERROR("Could not convert to image!");
	}
}

int main (int argc, char** argv){
	ros::init(argc, argv, "PlanarRegionPublisher");

	ros::NodeHandle n;
	RGBDPosePub = n.advertise<geometry_msgs::PoseStamped>("rgbd_pose", 1000);
	planarRegionPub = n.advertise<map_sense::PlanarRegion>("planar_regions", 1000);

	cv::namedWindow("DepthCallback");
	ros::Subscriber sub = n.subscribe("/depth_image/compressed", 1, depthCallback);


	// image_transport::ImageTransport it(n);
	// image_transport::TransportHints hints("compressed");
	// image_transport::Subscriber sub = it.subscribe("/depth_image", 1, depthCallback, ros::VoidPtr(), hints);





	ros::Rate loop_rate(200);

	int count = 0;
	int r = 3;

	while (ros::ok()){


		geometry_msgs::PoseStamped msg;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "/world";
		msg.pose.position.x = r*cos(count*0.01);
		msg.pose.position.y = r*sin(count*0.01);
		msg.pose.position.z = 3.0;
		msg.pose.orientation.x = 0.0;
		msg.pose.orientation.y = 0.3826834;
		msg.pose.orientation.z = 0.0;
		msg.pose.orientation.w = 0.9238795;
		RGBDPosePub.publish(msg);


		ros::spinOnce();

		loop_rate.sleep();
		++count;


	}
	cv::destroyWindow("DepthCallback");

	return 0;
}