#include <ros/ros.h>
#include "HSM_Heartbeat_Monitor_class.h"


int main(int argc, char **argv)
{
	ros::init(argc,argv,"HSM_HB_Executive");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(1);

	ros::spinOnce();

	HSM_Heartbeat_Monitor_class HSM_Heartbeat_Monitor("HSM_TEST_node_1", 1);
		
	ros::spinOnce();

	while(ros::ok())
	{
		HSM_Heartbeat_Monitor.service_monitor();
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

