#include <ros/ros.h>
#include <navigation/navigation_filter.hpp>
#include <hsm/user_input_nav_act_class.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigation_filter_node");
	ROS_INFO("navigation_filter_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	NavigationFilter navigationfilter;
	navigationfilter.state = navigationfilter.state_waiting;

	//User_Input_Nav_Act user_input_nav_act(&filter.x, &filter.P_x, &filter.y, &filter.P_y, &filter.psi, &filter.P_psi, &filter.north_angle, &filter.P_north_angle, &filter1.x, &filter1.P_x, &filter1.y, &filter1.P_y, &filter1.psi, &filter1.P_psi, &filter1.north_angle, &filter1.P_north_angle, &filter2.x, &filter2.P_x, &filter2.y, &filter2.P_y, &filter2.psi, &filter2.P_psi, &filter2.north_angle, &filter2.P_north_angle, &filter3.x, &filter3.P_x, &filter3.y, &filter3.P_y, &filter3.psi, &filter3.P_psi, &filter3.north_angle, &filter3.P_north_angle);

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