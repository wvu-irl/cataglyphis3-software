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
	
    ros::Rate loop_rate(600);

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

