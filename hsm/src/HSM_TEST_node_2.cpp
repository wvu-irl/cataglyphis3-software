#include <ros/ros.h>
#include "HSM_Heartbeat.h"


int main(int argc, char **argv)
{
	ros::init(argc,argv,"HSM_TEST_node_2");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(1);

	ros::spinOnce();

	HSM_Heartbeat hb;
		
	ros::spinOnce();

	while(ros::ok())
	{
		hb.Beat();
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

