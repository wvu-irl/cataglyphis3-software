#include <ros/ros.h>
#include "Comm_Monitor_class.h"
#include "Escape_Monitor_class.h"
#include "HSM_Heartbeat_Monitor_class.h"
#include <hsm/MasterStatus.h>
#include <comm/i7_to_nb1_msg.h>

int main(int argc, char **argv)
{
	ros::init(argc,argv,"HSM_Master_Executive");
	ros::NodeHandle nh;
	ros::Publisher master_status_pub = nh.advertise<hsm::MasterStatus>("hsm/masterstatus/masterstatus",1);
	ros::Publisher nb1_out_pub = nh.advertise<comm::i7_to_nb1_msg>("comm/nb1out/nb1out",1);
	hsm::MasterStatus master_status_msg;
	comm::i7_to_nb1_msg nb1_out_msg;
	
	ros::Rate loop_rate(100);

	ros::spinOnce();

	Comm_Monitor_class comm_monitor;
	Escape_Monitor_class escape_monitor;
	HSM_Heartbeat_Monitor_class vision_monitor("vision_state_machine_node_v2",100);
		
	ros::spinOnce();

	while(ros::ok())
	{
		ROS_INFO_THROTTLE(3,"HSM Master Executive Running");
		comm_monitor.service_monitor();
		escape_monitor.service_monitor();
		vision_monitor.service_monitor();
		
		master_status_msg.nb1_go = comm_monitor.nb1_go;
		master_status_msg.nb2_go = comm_monitor.nb2_go;
		master_status_msg.nb3_go = comm_monitor.nb3_go;
		master_status_pub.publish(master_status_msg);
		nb1_out_pub.publish(nb1_out_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

