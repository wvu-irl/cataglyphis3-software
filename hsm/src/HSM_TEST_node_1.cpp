#include <ros/ros.h>
#include "HSM_Heartbeat.h"


int main(int argc, char **argv)
{
	//ros::init(argc,argv,"HSM_TEST_node_1");
	ros::init(argc,argv, "vision_state_machine_node_v2");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(100);

	ros::spinOnce();

	HSM_Heartbeat hb;
		
	ros::spinOnce();
	int counter = 0;

	while(ros::ok())
	{
		if(counter>1000)
		{
			while(1) {usleep(100000); int j = 99;}
		}
		hb.Beat();
		ros::spinOnce();
		loop_rate.sleep();
		counter++;
	}
	
	return 0;
}

