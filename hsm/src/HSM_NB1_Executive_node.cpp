#include <ros/ros.h>
#include "HSM_NB1_Monitor_class.h"


int main(int argc, char **argv)
{
	ros::init(argc,argv,"HSM_NB1_Executive");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(100);

	ros::spinOnce();

	HSM_NB1_Monitor_class HSM_NB1_Monitor;
		
	ros::spinOnce();

	while(ros::ok())
	{
		HSM_NB1_Monitor.service_monitor();
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

