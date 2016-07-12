#ifndef PACKET_SUPERCLASS_H
#define PACKET_SUPERCLASS_H
#include <ros/ros.h>

class Packet
{
public:
	//members
	ros::Subscriber sub;
	ros::NodeHandle nh;
	uint16_t counter_value = 0;
	//methods
	virtual void unpackMsg()=0;
	virtual void subscribeMsg()=0;
	virtual void publishMsg(ros::Publisher* pub_ptr)=0;
	uint16_t incrementCounter();
};

uint16_t Packet::incrementCounter()
{
	counter_value++;
	return counter_value;
}

#endif /* PACKET_SUPERCLASS_H */
