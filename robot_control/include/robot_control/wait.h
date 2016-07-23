#ifndef WAIT_H
#define WAIT_H
#include "action.h"

class Wait : public Action
{
public:
	Wait(); // constructor
	void init();
	int run();
	ros::Timer waitTimer;
	bool waitTimerActive;
private:
	ros::NodeHandle nh;
	float waitTime_; // sec
	bool timeExpired_;
	void waitTimeCallback_(const ros::TimerEvent &event);
};

#endif // WAIT_H
