/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef CATAGLYPHIS_TIMER_H
#define CATAGLYPHIS_TIMER_H
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include "cataglyphis_timer_base.h"

template <class T>
class CataglyphisTimer : public CataglyphisTimerBase
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
private:
	// Methods
	boost::function <void(const ros::TimerEvent &)> externalCallback;
	void internalCallback(const ros::TimerEvent &event);
	// Members
	ros::NodeHandle nh;
	ros::Timer rosTimer;
	const float defaultDuration = 10.0; // sec
};

template <class T>
CataglyphisTimer<T>::CataglyphisTimer(void (T::*callback)(const ros::TimerEvent &), T *obj)
	: externalCallback(boost::bind(callback, obj, _1))
{
	rosTimer = nh.createTimer(ros::Duration(defaultDuration), &CataglyphisTimer::internalCallback, this, false, false);
	paused = false;
	running = false;
}

template <class T>
void CataglyphisTimer<T>::setPeriod(float periodIn)
{
	if(!paused)
	{
		period = periodIn;
		rosTimer.setPeriod(ros::Duration(period));
	}
}

template <class T>
void CataglyphisTimer<T>::adjustPeriod(float periodAdjust)
{
	rosTimer.stop();
	rosTimer.setPeriod(ros::Duration(period - (ros::Time::now().toSec() - startTime) + periodAdjust));
	rosTimer.start();
}

template <class T>
void CataglyphisTimer<T>::start()
{
	if(!running)
	{
	    setPeriod(period);
		rosTimer.start();
		startTime = ros::Time::now().toSec();
		paused = false;
		running = true;
	}
}

template <class T>
void CataglyphisTimer<T>::stop()
{
	if(running)
	{
		rosTimer.stop();
		running = false;
	}
}

template <class T>
void CataglyphisTimer<T>::pause()
{
	if(running && !paused)
	{
		pauseTime = ros::Time::now().toSec();
		rosTimer.stop();
		paused = true;
	}
}

template <class T>
void CataglyphisTimer<T>::resume()
{
	if(running && paused)
	{
		rosTimer.setPeriod(ros::Duration(period - (pauseTime - startTime)));
		rosTimer.start();
		paused = false;
	}
}

template <class T>
void CataglyphisTimer<T>::internalCallback(const ros::TimerEvent &event)
{
	rosTimer.stop();
	running = false;
	externalCallback(event);
}

#endif // CATAGLYPHIS_TIMER_H
