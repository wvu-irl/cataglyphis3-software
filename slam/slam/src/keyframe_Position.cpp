#include <slam/keyframe_Position.h>

void keyframe_Position::getkeyframe_PositionCallback(const slam::transformation_msg::ConstPtr &msg)
	{
		this->x = msg->x;
		this->y = msg->y;
		this->heading = msg->heading;
		counter++;
	}


keyframe_Position::keyframe_Position()
	{
		x=0;
		y=0;
		heading=0;
		counter=0;
		subscriber_nav = node.subscribe("keyframe_position", 1, &keyframe_Position::getkeyframe_PositionCallback,this);
	}
