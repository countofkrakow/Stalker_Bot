#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	double x = msg->pose.pose.position.x;
	double y = msg->pose.pose.position.y; 
	ROSINFO("x": %f, y %f, x, y);
}

int main(int argc, char** argv) { 
	ros::init(argc, argv, "location_monitor");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("odom", 10, OdomCalback);
	ros.spin();
	return 0;
} 
