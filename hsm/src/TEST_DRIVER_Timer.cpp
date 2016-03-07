#include "ros/ros.h"
#include <string>
#include "Timer.h"

void quickTimerCallback(const ros::TimerEvent&)
{
	static int cnt=0;
	cnt++;
	printf(" - - %d quick timer callback.\n", cnt);
}

void slowTimerCallback(const ros::TimerEvent&)
{
	static int cnt=0;
	cnt++;
	printf(" - %d slow timer callback.\n", cnt);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TEST_DRIVER");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);

	int loop_num=0;

	Timer quick(ros::Duration(0.1), &quickTimerCallback);
	Timer slow(ros::Duration(10), &slowTimerCallback);
	slow.start();
	quick.start();
	
//	while (ros::ok())
//	{
		loop_num++;
		printf("%d loop iteration...\n", loop_num);

//	    ros::spinOnce();
	    ros::spin();

    	loop_rate.sleep();
// 	}


	return 0;
}
