#include <ros/ros.h>
#include <navigation/navigation_filter.hpp>
#include <hsm/user_input_nav_act_class.h>
#include <messages/NavFilterOut.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigation_filter_node");
	ROS_INFO("navigation_filter_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	ros::Publisher pub = nh.advertise<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout",1);
	messages::NavFilterOut msg_NavFilterOut;

	NavigationFilter navigationfilter;
	navigationfilter.state = navigationfilter.state_waiting;

	User_Input_Nav_Act user_input_nav_act(&navigationfilter.filter.x, &navigationfilter.filter.P_x, &navigationfilter.filter.y, &navigationfilter.filter.P_y, &navigationfilter.filter.psi, &navigationfilter.filter.P_psi, &navigationfilter.filter.north_angle, &navigationfilter.filter.P_north_angle, &navigationfilter.filter1.x, &navigationfilter.filter1.P_x, &navigationfilter.filter1.y, &navigationfilter.filter1.P_y, &navigationfilter.filter1.psi, &navigationfilter.filter1.P_psi, &navigationfilter.filter1.north_angle, &navigationfilter.filter1.P_north_angle, &navigationfilter.filter2.x, &navigationfilter.filter2.P_x, &navigationfilter.filter2.y, &navigationfilter.filter2.P_y, &navigationfilter.filter2.psi, &navigationfilter.filter2.P_psi, &navigationfilter.filter2.north_angle, &navigationfilter.filter2.P_north_angle, &navigationfilter.filter3.x, &navigationfilter.filter3.P_x, &navigationfilter.filter3.y, &navigationfilter.filter3.P_y, &navigationfilter.filter3.psi, &navigationfilter.filter3.P_psi, &navigationfilter.filter3.north_angle, &navigationfilter.filter3.P_north_angle);

	while(ros::ok())
	{
		ROS_INFO("*-*-*-*-*-*-*-*-*-*-*-*");
		ROS_INFO("State = %i",navigationfilter.state);
		navigationfilter.update_time(); //updates dt and current_time
		switch(navigationfilter.state)
		{
			case 0: //_waiting
				navigationfilter.waiting(user_input_nav_act);
				break;
			case 1: //_forklift_drive
				navigationfilter.forklift_drive(user_input_nav_act);
				break;
			case 2: //_run
				navigationfilter.run(user_input_nav_act);
				break;
		}

		navigationfilter.filter.counter++; //this should be changed to function .increment_counter();
		navigationfilter.imu.set_prev_counters();
		
		msg_NavFilterOut.roll_rate = navigationfilter.imu.p;
		msg_NavFilterOut.pitch_rate = navigationfilter.imu.q;
		msg_NavFilterOut.yaw_rate = navigationfilter.imu.r;
		msg_NavFilterOut.ax = navigationfilter.imu.ax;
		msg_NavFilterOut.ay = navigationfilter.imu.ay;
		msg_NavFilterOut.az = navigationfilter.imu.az;
		//init_complete not used??
		msg_NavFilterOut.allSlipFlag=0;
		msg_NavFilterOut.counter=navigationfilter.filter.counter;
		msg_NavFilterOut.nav_status = navigationfilter.nav_status_output;
		msg_NavFilterOut.p1_offset = navigationfilter.imu.p1_offset;
		msg_NavFilterOut.q1_offset = navigationfilter.imu.q1_offset;
		msg_NavFilterOut.r1_offset = navigationfilter.imu.r1_offset;
		msg_NavFilterOut.p2_offset = navigationfilter.imu.p2_offset;
		msg_NavFilterOut.q2_offset = navigationfilter.imu.q2_offset;
		msg_NavFilterOut.r2_offset = navigationfilter.imu.r2_offset;
		msg_NavFilterOut.p3_offset = navigationfilter.imu.p3_offset;
		msg_NavFilterOut.q3_offset = navigationfilter.imu.q3_offset;
		msg_NavFilterOut.r3_offset = navigationfilter.imu.r3_offset;
		msg_NavFilterOut.dt = navigationfilter.dt;
		msg_NavFilterOut.x_position = navigationfilter.filter.x;
		msg_NavFilterOut.y_position = navigationfilter.filter.y;
		msg_NavFilterOut.roll = navigationfilter.filter.phi*180/3.1415927;
		msg_NavFilterOut.pitch = navigationfilter.filter.theta*180/3.1415927;
		msg_NavFilterOut.heading = navigationfilter.filter.psi*180/3.1415927;
		msg_NavFilterOut.human_heading = fmod(navigationfilter.filter.psi*180/3.1415927,360);
		msg_NavFilterOut.bearing = atan2(navigationfilter.filter.y,navigationfilter.filter.x)*180/3.1415927;
		msg_NavFilterOut.velocity = navigationfilter.encoders.delta_distance/navigationfilter.dt;
		msg_NavFilterOut.update = 0;
		msg_NavFilterOut.roll_init = navigationfilter.init_filter.phi*180.0/navigationfilter.PI;
		msg_NavFilterOut.pitch_init = navigationfilter.init_filter.theta*180.0/navigationfilter.PI;
		msg_NavFilterOut.heading_init = navigationfilter.init_filter.psi*180.0/navigationfilter.PI;
		msg_NavFilterOut.north_angle = navigationfilter.filter.north_angle*180.0/navigationfilter.PI; //128.0; // deg
		msg_NavFilterOut.homing_updated = navigationfilter.homing_updated;
		//msg_NavFilterOut.drop_off_distance = drop_off_dist;

		pub.publish(msg_NavFilterOut);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}