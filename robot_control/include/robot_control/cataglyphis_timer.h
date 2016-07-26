#ifndef CATAGLYPHIS_TIMER_H
#define CATAGLYPHIS_TIMER_H
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>

template <class T>
class CataglyphisTimer
{
public:
	// Methods
	CataglyphisTimer(void(T::*callback)(const ros::TimerEvent &), T *obj);
	void setPeriod(float periodIn); // sec
	void adjustPeriod(float periodAdjust); // sec
	void start();
	void stop();
	void pause();
	void resume();
	// Members
	float period;
	double startTime;
	double pauseTime;
	bool paused;
	bool running;
private:
	// Methods
	boost::function <void(const ros::TimerEvent &)> externalCallback;
	void internalCallback(const ros::TimerEvent &event);
	// Members
	ros::NodeHandle nh;
	ros::Timer rosTimer;
	const float defaultDuration = 10.0; // sec
};

#endif // CATAGLYPHIS_TIMER_H
