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

#ifndef HSM_DETECTOR_H
#define HSM_DETECTOR_H

#include <ros/ros.h>
#include <string>
#include <hsm/HSM_Detection.h>
//#include "Timer.h"


class HSM_Detector
{
private:
	// Members
	bool error_reported; //true if error reported and not returned to good
	// ROS
	ros::Timer watchdog;
	ros::Duration watchdogPeriod;
	// Methods
	void Init(std::string postfix);
	void HSM_timeout_callback(const ros::TimerEvent&);
	void watchdog_restart();

public:
	// Members
	std::string detector_node;
	std::string node_type;
	// ROS
	ros::Publisher pub;
	hsm::HSM_Detection msg;

	// Methods
	HSM_Detector(); // Constructor
	HSM_Detector(std::string const & postfix); // detector-extension Constructor
	HSM_Detector(ros::Duration TimeoutPeriod); // Constructor with timeout
	~HSM_Detector(); // Destructor
	void HSM_notify(std::string error_type);
	void HSM_good();
	void watchdog_setPeriod(ros::Duration);
	void watchdog_start();
};

#endif /* HSM_DETECTOR_H */
