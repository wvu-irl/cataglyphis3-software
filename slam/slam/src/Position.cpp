#include <slam/Position.h>

void Position::getPositionCallback(const messages::NavFilterOut::ConstPtr &msg)
	{
		this->x = msg->x_position;
		this->y = msg->y_position;
		this->heading = msg->heading;
		counter++;
//		ROS_INFO_STREAM("msg.x_position: " << msg->y_position);
	}


Position::Position()
	{
		x=0;
		y=0;
		heading=0;
		counter=0;
		subscriber_nav = node.subscribe("navigation/navigationfilterout/navigationfilterout", 1, &Position::getPositionCallback,this);
	}
