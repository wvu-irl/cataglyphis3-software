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

#include "ros/ros.h"
#include <string>
#include "Timer.h"

void quickTimerCallback(const ros::TimerEvent&)
{
	static int cnt=0;
	cnt++;
	printf(" - - %d quick timer callback.\n", cnt);
}

void slowTimerCallback(const ros::TimerEvent&)
{
	static int cnt=0;
	cnt++;
	printf(" - %d slow timer callback.\n", cnt);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TEST_DRIVER");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);

	int loop_num=0;

	Timer quick(ros::Duration(0.1), &quickTimerCallback);
	Timer slow(ros::Duration(10), &slowTimerCallback);
	slow.start();
	quick.start();

//	while (ros::ok())
//	{
		loop_num++;
		printf("%d loop iteration...\n", loop_num);

//	    ros::spinOnce();
	    ros::spin();

    	loop_rate.sleep();
// 	}


	return 0;
}
