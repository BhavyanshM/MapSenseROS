#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_sense/PlanarRegion.h"
#include "geometry_msgs/PoseStamped.h"


#include "math.h"

#include <sstream>

int main (int argc, char** argv){
	ros::init(argc, argv, "PlanarRegionPublisher");

	ros::NodeHandle n;

	ros::Publisher RGBDPosePub = n.advertise<geometry_msgs::PoseStamped>("rgbd_pose", 1000);
	ros::Publisher planarRegionPub = n.advertise<map_sense::PlanarRegion>("planar_regions", 1000);


	ros::Rate loop_rate(100);

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

		map_sense::PlanarRegion planeMsg;
		planeMsg.header.stamp = ros::Time::now();
		planeMsg.header.frame_id = "/world_of_planes";
		planeMsg.polygon.polygon.points.reserve(10);
		for(int i = 0; i<10; i++){
			geometry_msgs::Point32 point;
			point.x = 2.0*i;
			point.y = i;
			point.z = 3*i;
			planeMsg.polygon.polygon.points.push_back(point);
		}

		planarRegionPub.publish(planeMsg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}