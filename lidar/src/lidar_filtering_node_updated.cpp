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

#include <lidar/lidar_filtering.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_filtering_node");
	ROS_INFO("lidar_filtering_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(20);
	ros::Publisher pub_lidar = nh.advertise<messages::LidarFilterOut>("lidar/lidarfilteringout/lidarfilteringout",1);
	ros::Publisher pub_local_map = nh.advertise<messages::LocalMap>("/lidar/lidarfilteringnode/localmap",1);
	LidarFilter lidar_filter;
	messages::LidarFilterOut msg_LidarFilterOut;
	messages::LocalMap msg_LocalMap;

	while(ros::ok())
	{
		lidar_filter._homing_x=0;
		lidar_filter._homing_y=0;
		lidar_filter._homing_heading=0;
		lidar_filter._homing_found=false;

		if(lidar_filter.newPointCloudAvailable())
		{
			lidar_filter.doMathMapping();
			//lidar_filter.doMathHoming();
			if(lidar_filter._do_homing)
			{
				lidar_filter.stackClouds(); //note this function needs to go after doMathMapping
				lidar_filter.doLongDistanceHoming();
			}
			else
			{
				lidar_filter._stack_counter = 0;
			}
		}

		//if homing beacon is found reset stacking
		if(lidar_filter._homing_found == true)
		{
			lidar_filter._stack_counter = 0;
		}

		lidar_filter.packLocalMapMessage(msg_LocalMap);
		lidar_filter.packHomingMessage(msg_LidarFilterOut);
		lidar_filter.setPreviousCounters();

		pub_lidar.publish(msg_LidarFilterOut);
		pub_local_map.publish(msg_LocalMap);

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
