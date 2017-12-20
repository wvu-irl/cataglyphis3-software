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

#include "HSM_Detector.h"

HSM_Detector::HSM_Detector() // Constructor
{
	Init("");
}

HSM_Detector::HSM_Detector(std::string const & postfix) // detector-extension Constructor
{
	Init("/"+postfix);
}

HSM_Detector::HSM_Detector(ros::Duration TimeoutPeriod) // Constructor with timeout
{
	Init("");
	ros::NodeHandle nh;
	watchdogPeriod = TimeoutPeriod;
	watchdog =  nh.createTimer(watchdogPeriod, &HSM_Detector::HSM_timeout_callback, this);
	watchdog.stop(); // stop newly created timer until ready to time
}

HSM_Detector::~HSM_Detector() // Destructor
{
//	if(watchdog!=0){delete &watchdog;}
}

void HSM_Detector::Init(std::string postfix)
{
	ros::NodeHandle n;
	std::string node_name = ros::this_node::getName();
	pub = n.advertise<hsm::HSM_Detection>("HSM_Det/"+node_name+postfix,1);
	//ROS_INFO("HSM: %s publisher advertised.", pub.getTopic());
	msg.detector_node = node_name;
	error_reported = false;
//	watchdog = 0;
}

void HSM_Detector::HSM_notify(std::string error_type)
{
	if(watchdog!=0){watchdog_restart();}
	msg.message_type = error_type;
	pub.publish(msg);
	error_reported = true;
}

void HSM_Detector::HSM_good()
{
	if(watchdog!=0){watchdog_restart();}
	if (error_reported)
	{
		msg.message_type = "GOOD";
		pub.publish(msg);
		error_reported = false;
	}
}

void HSM_Detector::HSM_timeout_callback(const ros::TimerEvent& event)
{
	msg.message_type = "TIMEOUT";
	pub.publish(msg);
	error_reported = true;
	watchdog_restart();
}

void HSM_Detector::watchdog_start()
{
	watchdog.setPeriod(watchdogPeriod);
	watchdog.start();
}

void HSM_Detector::watchdog_restart()
{
	watchdog.stop();
	watchdog.setPeriod(watchdogPeriod);
	watchdog.start();
}

void HSM_Detector::watchdog_setPeriod(ros::Duration newTimerPeriod)
{
	watchdogPeriod = newTimerPeriod;
}
