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
#include "Comm_Monitor_class.h"
#include "Escape_Monitor_class.h"
//#include "HSM_Heartbeat_Monitor_class.h"
#include <robot_pose_monitor.h>
#include <messages/MasterStatus.h>

int main(int argc, char **argv)
{
	ros::init(argc,argv,"HSM_Master_Executive");
	ros::NodeHandle nh;
	ros::Publisher master_status_pub = nh.advertise<messages::MasterStatus>("hsm/masterexecutive/masterstatus",1);
	messages::MasterStatus master_status_msg;

	ros::Rate loop_rate(100);

    //Comm_Monitor_class comm_monitor;
	Escape_Monitor_class escape_monitor;
	//HSM_Heartbeat_Monitor_class vision_monitor("vision_state_machine_node_v2",100);
	RobotPoseMonitor robotPoseMonitor;

	ros::Duration(1).sleep();
	ros::spinOnce();

	while(ros::ok())
	{
		ROS_INFO_THROTTLE(3,"HSM Master Executive Running");
        //comm_monitor.service_monitor();
		escape_monitor.service_monitor();
		//vision_monitor.service_monitor();
		// robotPoseMonitor serviced by internal timer at 20 Hz
        master_status_msg.navSolutionsDiverged = robotPoseMonitor.navSolutionsDiverged;
		master_status_msg.lidar_go = true; // Need to implement monitor to evaluate this
		master_status_msg.zed_go = false; // ^^^
		master_status_pub.publish(master_status_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
