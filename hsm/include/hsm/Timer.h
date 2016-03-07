#ifndef TIMER_H
#define TIMER_H

#include <ros/ros.h>

typedef void (*timerCallback_t)(const ros::TimerEvent&); //typedef for timer callback function pointer

class Timer
{
private:
	// Members
	ros::Duration timerPeriod;
	ros::Timer timer;

public:
	// Methods
	Timer(ros::Duration, timerCallback_t); // Constructor for function callback
//	Timer(ros::Duration, timerCallback_t, void*); // Constructor object method callback
	~Timer(); // Destructor
	void start();
	void stop();
	void restart();
	void setPeriod(ros::Duration);
};

#endif /* TIMER_H */
