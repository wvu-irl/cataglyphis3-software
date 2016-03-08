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

	User_Input_Nav_Act user_input_nav_act(&navigationfilter.filter.x, &navigationfilter.filter.P_x, &navigationfilter.filter.y, &navigationfilter.filter.P_y, &navigationfilter.filter.psi, &navigationfilter.filter.P_psi, &navigationfilter.filter.north_angle, &navigationfilter.filter.P_north_angle, &navigationfilter.filter1.x, &navigationfilter.filter1.P_x, &navigationfilter.filter1.y, &navigationfilter.filter1.P_y, &navigationfilter.filter1.psi, &navigationfilter.filter1.P_psi, &navigationfilter.filter1.north_angle, &navigationfilter.filter1.P_north_angle, &navigationfilter.filter2.x, &navigationfilter.filter2.P_x, &navigationfilter.filter2.y, &navigationfilter.filter2.P_y, &navigationfilter.filter2.psi, &navigationfilter.filter2.P_psi, &navigationfilter.filter2.north_angle, &navigationfilter.filter2.P_north_angle, &navigationfilter.filter3.x, &navigationfilter.filter3.P_x, &navigationfilter.filter3.y, &navigationfilter.filter3.P_y, &navigationfilter.filter3.psi, &navigationfilter.filter3.P_psi, &navigationfilter.filter3.north_angle, &navigationfilter.filter3.P_north_angle);

	while(ros::ok())
	{
		ROS_INFO("*-*-*-*-*-*-*-*-*-*-*-*");
		ROS_INFO("State = %i",navigationfilter.state);
		navigationfilter.update_time(); //updates dt and current_time
		switch(navigationfilter.state)
		{
			case 0: //_waiting
				navigationfilter.waiting(user_input_nav_act);
				break;
			case 1: //_forklift_drive
				navigationfilter.forklift_drive(user_input_nav_act);
				break;
			case 2: //_run
				navigationfilter.run(user_input_nav_act);
				break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}