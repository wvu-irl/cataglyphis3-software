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

#include <lidar/collision_detection.hpp>

//test
#include <lidar/telemetry.hpp>

//test
// #include <time.h>	// show calculation time


int main(int argc, char **argv)
{
	ros::init(argc, argv, "collision_filtering_node");
	ROS_INFO("collision_filtering_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(20);
	ros::Publisher pub_col = nh.advertise<messages::CollisionOut>("lidar/collisiondetectionout/collisiondetectionout",1);
	CollisionDetection collision_detection;
	messages::CollisionOut msg_CollisionOut;
	collision_detection.Initializations();

	//timer definition, for testing
	// clock_t start, finish;
	// double totaltime;
	delta_loop_time telemetry;

	while(ros::ok())
	{
		//timer, debug
		// start = clock();
		// telemetry.printMetrics(true);

		if(collision_detection.newPointCloudAvailable())
		{
			collision_detection.doMathSafeEnvelope();
			//collision_detection.doMathRANSAC();
			//collision_detection.doPredictiveAovidance();
			collision_detection.packCollisionMessage(msg_CollisionOut);
		}
		collision_detection.setPreviousCounters();
		pub_col.publish(msg_CollisionOut);
		loop_rate.sleep();
		ros::spinOnce();

		// finish = clock();
		// totaltime = (double)(finish - start) / CLOCKS_PER_SEC;

		// ROS_INFO_STREAM("time: " << totaltime << "s");
	}

	return 0;
}
