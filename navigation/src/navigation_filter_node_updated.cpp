#include <ros/ros.h>
#include <navigation/navigation_filter.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigation_filter_node");
	ROS_INFO("navigation_filter_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	NavigationFilter navigationfilter;

	enum state_t {_waiting, _forklift_drive, _run};
	static state_t state = _waiting;

	while(ros::ok())
	{
		navigationfilter.update_time(); //updates dt and current_time
		switch(state)
		{
			case _waiting:
				navigationfilter.waiting();
				break;
			case _forklift_drive:
				navigationfilter.forklift_drive();
				break;
			case _run:
				navigationfilter.run();
				break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}