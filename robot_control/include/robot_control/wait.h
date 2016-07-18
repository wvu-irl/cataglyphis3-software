#ifndef WAIT_H
#define WAIT_H
#include "action.h"

class Wait : public Action
{
public:
	void init();
	int run();
private:
	float waitTime_; // sec
	ros::Timer timer;
	bool timeExpired_;
	void waitTimeCallback_(const ros::TimerEvent &event);
};

#endif // WAIT_H
