#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "comm/ROS_Node_Superclass.h"
#include "HSM_Detector.h"
#include <string>
#include <sstream>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "TEST_DRIVER_HSM_Detector");
	ros::NodeHandle n;
	std::string err_type;	


	HSM_Detector det(ros::Duration(0.5));
	det.watchdog_start();
	
	ros::Rate loop_rate(100);

	int count = 0;
	err_type = "BAD PACKET";

	while (ros::ok())
	{

		std::cout << count << std::endl;

		if(count<1500){det.HSM_good();}
		if(count==2500){det.HSM_good();}
		if(count>3000){det.HSM_notify(err_type);}

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}


	return 0;
}

