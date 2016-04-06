#include "ros/ros.h"
#include "ros/console.h"

#include <slam/transformation_msg.h>	//change depend on path

class keyframe_Position
{
private:
	ros::Subscriber subscriber_nav;
	ros::NodeHandle node;
	
	//navigation callback for deadreckoning position
	void getkeyframe_PositionCallback(const slam::transformation_msg::ConstPtr &msg);

public:
	float x,y,heading;
	int counter;
	keyframe_Position();
};