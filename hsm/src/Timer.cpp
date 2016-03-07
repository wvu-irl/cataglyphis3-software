#include "Timer.h"
#include <stdio.h>

Timer::Timer(ros::Duration setTimerPeriod, timerCallback_t timerCallback) // Constructor for callback function
{
	ros::NodeHandle nh;
	timerPeriod = setTimerPeriod;
	timer = nh.createTimer(timerPeriod,timerCallback/*,true*/);
	timer.stop(); // stop newly created timer until ready to time
}

/*
Timer::Timer(ros::Duration setTimerPeriod, timerCallback_t timerCallback, void* obj_ptr) // Constructor for callback object method
{
	ros::NodeHandle nh;
	timerPeriod = setTimerPeriod;
	timer = nh.createTimer(timerPeriod, timerCallback, obj_ptr);
	timer.stop(); // stop newly created timer until ready to time
}
*/

Timer::~Timer() // Destructor
{
	delete &timer;
}

void Timer::start()
{
	timer.setPeriod(timerPeriod);
	timer.start();
}

void Timer::stop()
{
	timer.stop();
}

void Timer::restart()
{
	timer.stop();
	timer.setPeriod(timerPeriod);
	timer.start();	
}

void Timer::setPeriod(ros::Duration newTimerPeriod)
{
	timerPeriod = newTimerPeriod;
}

