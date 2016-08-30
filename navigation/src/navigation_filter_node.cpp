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

	User_Input_Nav_Act user_input_nav_act(&navigationfilter.filter.x, &navigationfilter.filter.P_x, &navigationfilter.filter.y, &navigationfilter.filter.P_y, &navigationfilter.filter.psi, &navigationfilter.filter.P_psi, &navigationfilter.filter.north_angle, &navigationfilter.filter.P_north_angle, &navigationfilter.filter1.x, &navigationfilter.filter1.P_x, &navigationfilter.filter1.y, &navigationfilter.filter1.P_y, &navigationfilter.filter1.psi, &navigationfilter.filter1.P_psi, &navigationfilter.filter1.north_angle, &navigationfilter.filter1.P_north_angle, &navigationfilter.filter2.x, &navigationfilter.filter2.P_x, &navigationfilter.filter2.y, &navigationfilter.filter2.P_y, &navigationfilter.filter2.psi, &navigationfilter.filter2.P_psi, &navigationfilter.filter2.north_angle, &navigationfilter.filter2.P_north_angle);

	while(ros::ok())
	{
		//ROS_INFO("*-*-*-*-*-*-*-*-*-*-*-*");
		//ROS_INFO("State = %i",navigationfilter.state);
		navigationfilter.filter.counter++; //this should be changed to function .increment_counter();
		navigationfilter.update_time(); //updates dt and current_time
		navigationfilter.imu.determine_new_data();
		navigationfilter.imu.filter_imu_values();
		navigationfilter.imu.set_prev_counters();
		navigationfilter.encoders.adjustEncoderWrapError();
		navigationfilter.encoders.calculateWheelDistancesFromEncoders();
		navigationfilter.encoders.calculateDeltaDistance6Wheels(0, 0); //turnFlag, stopFlag
		switch(navigationfilter.state)
		{
			case _waiting: //_waiting
				navigationfilter.waiting(user_input_nav_act);
				break;
			case _forklift_drive: //_forklift_drive
				navigationfilter.forklift_drive(user_input_nav_act);
				break;
			case _run: //_run
				navigationfilter.run(user_input_nav_act);
				break;
		}


		msg_NavFilterOut.new_imu1 = navigationfilter.imu.new_imu1;
		msg_NavFilterOut.p1 = navigationfilter.imu.p1;
		msg_NavFilterOut.q1 = navigationfilter.imu.q1;
		msg_NavFilterOut.r1 = navigationfilter.imu.r1;
		msg_NavFilterOut.ax1 = navigationfilter.imu.ax1;
		msg_NavFilterOut.ay1 = navigationfilter.imu.ay1;
		msg_NavFilterOut.az1 = navigationfilter.imu.az1;
		msg_NavFilterOut.p1_offset = navigationfilter.imu.p1_offset;
		msg_NavFilterOut.q1_offset = navigationfilter.imu.q1_offset;
		msg_NavFilterOut.r1_offset = navigationfilter.imu.r1_offset;
		msg_NavFilterOut.new_imu2 = navigationfilter.imu.new_imu2;
		msg_NavFilterOut.p2 = navigationfilter.imu.p2;
		msg_NavFilterOut.q2 = navigationfilter.imu.q2;
		msg_NavFilterOut.r2 = navigationfilter.imu.r2;
		msg_NavFilterOut.ax2 = navigationfilter.imu.ax2;
		msg_NavFilterOut.ay2 = navigationfilter.imu.ay2;
		msg_NavFilterOut.az2 = navigationfilter.imu.az2;
		msg_NavFilterOut.p2_offset = navigationfilter.imu.p2_offset;
		msg_NavFilterOut.q2_offset = navigationfilter.imu.q2_offset;
		msg_NavFilterOut.r2_offset = navigationfilter.imu.r2_offset;
		msg_NavFilterOut.new_imu3 = navigationfilter.imu.new_imu3;
		msg_NavFilterOut.p3 = navigationfilter.imu.p3;
		msg_NavFilterOut.q3 = navigationfilter.imu.q3;
		msg_NavFilterOut.r3 = navigationfilter.imu.r3;
		msg_NavFilterOut.ax3 = navigationfilter.imu.ax3;
		msg_NavFilterOut.ay3 = navigationfilter.imu.ay3;
		msg_NavFilterOut.az3 = navigationfilter.imu.az3;
		msg_NavFilterOut.p3_offset = navigationfilter.imu.p3_offset;
		msg_NavFilterOut.q3_offset = navigationfilter.imu.q3_offset;
		msg_NavFilterOut.r3_offset = navigationfilter.imu.r3_offset;
		msg_NavFilterOut.new_imu4 = navigationfilter.imu.new_imu4;
		msg_NavFilterOut.p4 = navigationfilter.imu.p4;
		msg_NavFilterOut.q4 = navigationfilter.imu.q4;
		msg_NavFilterOut.r4 = navigationfilter.imu.r4;
		msg_NavFilterOut.ax4 = navigationfilter.imu.ax4;
		msg_NavFilterOut.ay4 = navigationfilter.imu.ay4;
		msg_NavFilterOut.az4 = navigationfilter.imu.az4;
		msg_NavFilterOut.p4_offset = navigationfilter.imu.p4_offset;
		msg_NavFilterOut.q4_offset = navigationfilter.imu.q4_offset;
		msg_NavFilterOut.r4_offset = navigationfilter.imu.r4_offset;
		msg_NavFilterOut.new_imu5 = navigationfilter.imu.new_imu5;
		msg_NavFilterOut.p5 = navigationfilter.imu.p5;
		msg_NavFilterOut.q5 = navigationfilter.imu.q5;
		msg_NavFilterOut.r5 = navigationfilter.imu.r5;
		msg_NavFilterOut.ax5 = navigationfilter.imu.ax5;
		msg_NavFilterOut.ay5 = navigationfilter.imu.ay5;
		msg_NavFilterOut.az5 = navigationfilter.imu.az5;
		msg_NavFilterOut.p5_offset = navigationfilter.imu.p5_offset;
		msg_NavFilterOut.q5_offset = navigationfilter.imu.q5_offset;
		msg_NavFilterOut.r5_offset = navigationfilter.imu.r5_offset;
		msg_NavFilterOut.new_imu6 = navigationfilter.imu.new_imu6;
		msg_NavFilterOut.p6 = navigationfilter.imu.p6;
		msg_NavFilterOut.q6 = navigationfilter.imu.q6;
		msg_NavFilterOut.r6 = navigationfilter.imu.r6;
		msg_NavFilterOut.ax6 = navigationfilter.imu.ax6;
		msg_NavFilterOut.ay6 = navigationfilter.imu.ay6;
		msg_NavFilterOut.az6 = navigationfilter.imu.az6;
		msg_NavFilterOut.p6_offset = navigationfilter.imu.p6_offset;
		msg_NavFilterOut.q6_offset = navigationfilter.imu.q6_offset;
		msg_NavFilterOut.r6_offset = navigationfilter.imu.r6_offset;
		msg_NavFilterOut.new_nb1 = navigationfilter.imu.new_nb1;
		msg_NavFilterOut.time1 = navigationfilter.imu.time1;
		msg_NavFilterOut.prev_time1 = navigationfilter.imu.prev_time1;
		msg_NavFilterOut.dt1 = navigationfilter.imu.dt1;
		msg_NavFilterOut.nb1_current = navigationfilter.imu.nb1_current;
		msg_NavFilterOut.nb1_missed_counter = navigationfilter.imu.nb1_missed_counter;
		msg_NavFilterOut.nb1_drive_counter = navigationfilter.imu.nb1_drive_counter;
		msg_NavFilterOut.nb1_diff_prev = navigationfilter.imu.nb1_diff_prev;
		msg_NavFilterOut.nb1_good = navigationfilter.imu.nb1_good;
		msg_NavFilterOut.nb1_good_prev = navigationfilter.imu.nb1_good_prev;
		msg_NavFilterOut.new_nb2 = navigationfilter.imu.new_nb2;
		msg_NavFilterOut.time2 = navigationfilter.imu.time2;
		msg_NavFilterOut.prev_time2 = navigationfilter.imu.prev_time2;
		msg_NavFilterOut.dt2 = navigationfilter.imu.dt2;
		msg_NavFilterOut.nb2_current = navigationfilter.imu.nb2_current;
		msg_NavFilterOut.nb2_missed_counter = navigationfilter.imu.nb2_missed_counter;
		msg_NavFilterOut.nb2_drive_counter = navigationfilter.imu.nb2_drive_counter;
		msg_NavFilterOut.nb2_diff_prev = navigationfilter.imu.nb2_diff_prev;
		msg_NavFilterOut.nb2_good = navigationfilter.imu.nb2_good;
		msg_NavFilterOut.nb2_good_prev = navigationfilter.imu.nb2_good_prev;
		msg_NavFilterOut.x_position1 = navigationfilter.filter1.x;
		msg_NavFilterOut.y_position1 = navigationfilter.filter1.y;
		msg_NavFilterOut.roll1 = navigationfilter.filter1.phi*180/3.1415927; 
		msg_NavFilterOut.pitch1 = navigationfilter.filter1.theta*180/3.1415927;
		msg_NavFilterOut.heading1 = navigationfilter.filter1.psi*180/3.1415927;
		msg_NavFilterOut.x_position2 = navigationfilter.filter2.x;
		msg_NavFilterOut.y_position2 = navigationfilter.filter2.y;
		msg_NavFilterOut.roll2 = navigationfilter.filter2.phi*180/3.1415927; 
		msg_NavFilterOut.pitch2 = navigationfilter.filter2.theta*180/3.1415927;
		msg_NavFilterOut.heading2 = navigationfilter.filter2.psi*180/3.1415927;
        msg_NavFilterOut.roll_rate = navigationfilter.imu.p;
        msg_NavFilterOut.pitch_rate = navigationfilter.imu.q;
        msg_NavFilterOut.yaw_rate = navigationfilter.imu.r;
		msg_NavFilterOut.x_position = navigationfilter.filter.x;
		msg_NavFilterOut.y_position = navigationfilter.filter.y;
		msg_NavFilterOut.roll = navigationfilter.filter.phi*180/3.1415927; 
		msg_NavFilterOut.pitch = navigationfilter.filter.theta*180/3.1415927;
		msg_NavFilterOut.heading = navigationfilter.filter.psi*180/3.1415927;
		msg_NavFilterOut.human_heading = fmod(navigationfilter.filter.psi*180/3.1415927,360);
		msg_NavFilterOut.bearing = atan2(navigationfilter.filter.y,navigationfilter.filter.x)*180/3.1415927;
		msg_NavFilterOut.velocity = navigationfilter.encoders.delta_distance/navigationfilter.dt;
		msg_NavFilterOut.counter=navigationfilter.filter.counter;
		msg_NavFilterOut.nav_status = navigationfilter.nav_status_output;
		msg_NavFilterOut.dt = navigationfilter.dt;
		msg_NavFilterOut.roll_init = navigationfilter.init_filter.phi*180.0/navigationfilter.PI;
		msg_NavFilterOut.pitch_init = navigationfilter.init_filter.theta*180.0/navigationfilter.PI;
		msg_NavFilterOut.heading_init = navigationfilter.init_filter.psi*180.0/navigationfilter.PI;
		msg_NavFilterOut.north_angle = navigationfilter.filter.north_angle*180.0/navigationfilter.PI; //128.0; // deg+
		msg_NavFilterOut.Kens_north_angle = navigationfilter.filter.Kens_north_angle*180.0/navigationfilter.PI; //128.0; // deg+
		msg_NavFilterOut.Kens_angle = navigationfilter.filter.Kens_angle*180.0/navigationfilter.PI; //128.0; // deg+
        msg_NavFilterOut.platform_number = navigationfilter.filter.platform_number;
		msg_NavFilterOut.homing_updated = navigationfilter.homing_updated;
		msg_NavFilterOut.stop_request = navigationfilter.stop_request;
		msg_NavFilterOut.stop_request_time = ros::Time::now().toSec()-navigationfilter.stop_time;
		msg_NavFilterOut.do_homing = navigationfilter.do_homing;

		pub.publish(msg_NavFilterOut);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
