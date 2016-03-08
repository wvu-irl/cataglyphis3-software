#include <ros/ros.h>
#include <navigation/navigation_filter.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigation_filter_node");
	ROS_INFO("navigation_filter_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	NavigationFilter navigationfilter;
	navigationfilter.state = navigationfilter.state_waiting;

	while(ros::ok())
	{
		navigationfilter.update_time(); //updates dt and current_time
		switch(navigationfilter.state)
		{
			case 0: //_waiting
				navigationfilter.waiting();
				break;
			case 1: //_forklift_drive
				navigationfilter.forklift_drive();
				break;
			case 2: //_run
				navigationfilter.run();
				break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}