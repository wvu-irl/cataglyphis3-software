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

#include <ros/ros.h>
#include "roboteq_class.h"

int main(int argc, char **argv)
{
    // Node Initialization
    ros::init(argc, argv, "speed_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);
    static int unresponsive_count = 0;
    const int min_speed_unresponsive = 250;
    const int min_delta_enc_unresponsive = 25;
    const int max_unresponsive_count = 20;
	double current_time;
	const double timeout_time = 1.0; // Seconds

    Roboteq_Class roboteq;

    while(ros::ok())
    {
    	ROS_INFO_THROTTLE(3,"Speed Node Running");
		current_time = ros::Time::now().toSec();
		if((current_time-roboteq.callback_time)>=timeout_time)
		{
			ROS_WARN("ROBOTEQ Command timeout time delay %f", current_time-roboteq.callback_time);
			roboteq.motor_1_speed_cmd = 0;
			roboteq.motor_2_speed_cmd = 0;
			roboteq.motor_3_speed_cmd = 0;
		}

		roboteq.readEncoderValues();

    	if(abs(roboteq.motor_1_speed_cmd)>=min_speed_unresponsive && abs(roboteq.motor_1_encoder_count-roboteq.motor_1_encoder_count_prev)<min_delta_enc_unresponsive)
    	{
    		unresponsive_count++;
    	}
    	else if(abs(roboteq.motor_2_speed_cmd)>=min_speed_unresponsive  && abs(roboteq.motor_2_encoder_count-roboteq.motor_2_encoder_count_prev)<min_delta_enc_unresponsive)
    	{
    		unresponsive_count++;
    	}
    	else
    	{
    		unresponsive_count=0;
    	}

    	if(0/*unresponsive_count>max_unresponsive_count*/)
    	{
    		ROS_ERROR("ROBOTEQ unresponsive based on encoder feedback");
    		exit(1);
    	}

    	roboteq.setMotorSpeeds();
    	ros::Duration(0.001).sleep();
    	roboteq.clearPluses();

    	roboteq.setPrevValues();
    	ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
