#include <ros/ros.h>
#include "roboteq_class.h"
#include "bit_utils.h"

Leading_Edge_Latch latch;

int main(int argc, char **argv)
{
    // Node Initialization
    ros::init(argc, argv, "grabber_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);  
    
    Roboteq_Class roboteq;
    
    while(ros::ok())
    {	
    	ROS_INFO_THROTTLE(3,"Grabber Node Running");
    	latch.LE_Latch(roboteq.grabber_stop_cmd);
    	if(latch.get_val()==1) roboteq.grabberStop();
    	else if(roboteq.grabber_stop_cmd==1&&latch.get_val()!=1);
		else roboteq.setGrabberCommands();
		ros::Duration(0.001).sleep();
		roboteq.readGrabberFeedback(); 
    	  	
    	ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
