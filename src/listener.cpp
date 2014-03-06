#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "ardrone_autonomy/Navdata.h"

ardrone_autonomy::Navdata msg;
float altd;
	
void nav_callback(const ardrone_autonomy::Navdata& msg)
{
	std::ostringstream oss;
	altd = msg.altd;
	oss << "Receive: " << altd;
	ROS_INFO(oss.str().c_str());
}

int main(int argc, char** argv)
{

	ROS_INFO("Starting...");
	ros::init(argc, argv,"ps_ardrone");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
	ros::Subscriber nav_sub;

	nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);
	
	ros::spin();
	
	return 0;
}
