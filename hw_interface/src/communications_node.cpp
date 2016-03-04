#include <ros/ros.h>
#include "Comm_class.h"

using namespace std;

int main(int argc, char **argv)
{
	std::string node_type;
	if(ros::param::get("node_type", node_type)==false) node_type = "comm_node";
	ROS_INFO("main() start");
	ros::init(argc, argv, node_type, ros::init_options::AnonymousName);
	ROS_INFO(" - ros::init complete");
	ros::NodeHandle nh;
	ROS_INFO(" - node handle created");
	
	Comm comm_obj;
	ROS_INFO(" - Comm object instantiated");
    ros::spinOnce();
	while(ros::ok())
	{
		ROS_INFO(" - - while loop in main()");
		comm_obj.run();
		ROS_INFO(" - - after comm_obj.run()");
	}
	return 0;
}
