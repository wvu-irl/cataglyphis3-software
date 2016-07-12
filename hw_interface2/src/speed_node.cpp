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
