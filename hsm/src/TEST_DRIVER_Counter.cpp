#include "ros/ros.h"
#include <string>
#include "Counter.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TEST_DRIVER");

	ros::NodeHandle n;

	ros::Rate loop_rate(50);

	Counter c;
	
	while (ros::ok())
	{
		printf("%d\n", c());
		if(c()%2==1) {c++;}
		else {c.increment();}
		if(c()==101) {c.reset();}
		
	    ros::spinOnce();

    	loop_rate.sleep();
  	}


	return 0;
}
