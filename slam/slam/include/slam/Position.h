#include "ros/ros.h"
#include "ros/console.h"
#include "messages/NavFilterOut.h"
//using namespace std;

class Position
{
private:
	ros::Subscriber subscriber_nav;
	ros::NodeHandle node;
	
	//navigation callback for deadreckoning position
	void getPositionCallback(const messages::NavFilterOut::ConstPtr &msg);

public:
	float x,y,heading;
	int counter;
	Position();
};