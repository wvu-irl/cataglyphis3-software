#include <navigation/navigation_filter.hpp>

NavigationFilter::NavigationFilter()
{
	sub_exec = nh.subscribe("/topicname", 1, &NavigationFilter::getExecInfoCallback, this);
	pause_switch = false;
	stopFlag = false;
	turnFlag = false;
	current_time = ros::Time::now().toSec();
}

void NavigationFilter::update_time()
{
	dt = ros::Time::now().toSec() - current_time;
	current_time = ros::Time::now().toSec();
}

void NavigationFilter::waiting(User_Input_Nav_Act user_input_nav_act)
{
	if (user_input_nav_act.bias_removal_forklift!=1)
	{
		first_pass = true;
		ROS_INFO("user_input_nav_act.bias_removal_forklift!=1");
	}
	else
	{
		ROS_INFO("user_input_nav_act.bias_removal_forklift==1");
	}
	if (first_pass == true)
	{
		filter.clear_accelerometer_values();
		imu.clear_gyro1_values();
		imu.clear_gyro2_values();
		imu.clear_gyro3_values();
		prev_stopped = false;
		collecting_accelerometer_data = false;
		collected_gyro_data = false;
		collected_gyro1_data = false;
		collected_gyro2_data = false;
		collected_gyro3_data = false;
		first_pass = false;
	}
	imu.determine_new_data();
	imu.filter_imu_values();
	if ((fabs(sqrt(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az)-1)< 0.05 && sqrt((imu.p)*(imu.p)+(imu.q)*(imu.q)+(imu.r)*(imu.r))<0.01) && encoders.delta_distance == 0)
	{
		if (collecting_accelerometer_data)
		{
			if (filter.ax_values.size() > 200)
			{
				collecting_accelerometer_data = false;
				filter.roll_pitch_G_update();
				filter.clear_accelerometer_values();
			}
			else
			{
				filter.collect_accelerometer_data(imu.ax, imu.ay, imu.az);
			}
		}
		if (user_input_nav_act.bias_removal_forklift==1)
		{
			if (imu.p1_values.size() > 500 && collected_gyro1_data!=true)
			{
				collected_gyro1_data = true;
				imu.calculate_gyro1_offset();
				imu.set_gyro1_offset();
			}
			else if (collected_gyro1_data==true)
			{
				collected_gyro1_data = true;
			}
			else
			{
				imu.collect_gyro1_data();
				collected_gyro1_data = false;
			}
			if (imu.p2_values.size() > 500 && collected_gyro2_data!=true)
			{
				collected_gyro2_data = true;
				imu.calculate_gyro2_offset();
				imu.set_gyro2_offset();
			}
			else if (collected_gyro2_data==true)
			{
				collected_gyro2_data = true;
			}
			else
			{
				imu.collect_gyro2_data();
				collected_gyro2_data = false;
			}
			if (imu.p3_values.size() > 500 && collected_gyro3_data!=true)
			{
				collected_gyro3_data = true;
				imu.calculate_gyro3_offset();
				imu.set_gyro3_offset();
			}
			else if (collected_gyro3_data==true)
			{
				collected_gyro3_data = true;
			}
			else
			{
				imu.collect_gyro3_data();
				collected_gyro3_data = false;
			}
			if (collected_gyro1_data && collected_gyro2_data && collected_gyro3_data)
			{
				collected_gyro_data = true;
			}
			else
			{
				collected_gyro_data = false;
			}
		}
	}
	if (user_input_nav_act.bias_removal_forklift==1 && collected_gyro_data)
	{
		nav_status_output = 1;
	}
	else
	{
		nav_status_output = 0;
	}
	imu.set_prev_counters();
	if(user_input_nav_act.begin_dead_reckoning==1) 
	{
		state = _forklift_drive;
		init_filter.initialize_states(filter.phi,filter.theta,user_input_nav_act.north_angle_init,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		filter1.initialize_states(filter.phi,filter.theta,user_input_nav_act.north_angle_init,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		filter2.initialize_states(filter.phi,filter.theta,user_input_nav_act.north_angle_init,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		filter3.initialize_states(filter.phi,filter.theta,user_input_nav_act.north_angle_init,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		init_filter.initialize_variance(collected_gyro_data,2.0*PI/180.0); //performed bias removal, north_angle_unc
		first_pass = true;
	}
	else state = _waiting;
	calibrate_counter++;
}

void NavigationFilter::forklift_drive(User_Input_Nav_Act user_input_nav_act)
{
	encoders.adjustEncoderWrapError();
	encoders.calculateWheelDistancesFromEncoders();
	encoders.calculateDeltaDistance6Wheels(0, 0); //turnFlag, stopFlag
	imu.determine_new_data();
	imu.filter_imu_values();
	if (imu.new_imu1!=0)
	{
		filter1.turning(imu.p1,imu.q1,imu.r1,imu.dt1);
	}
	else
	{
		filter1.blind_turning(imu.p1,imu.q1,imu.r1,imu.dt1);
	}
	if (imu.new_imu2!=0)
	{
		filter2.turning(imu.p2,imu.q2,imu.r2,imu.dt2);
	}
	else
	{
		filter2.blind_turning(imu.p2,imu.q2,imu.r2,imu.dt2);
	}
	if (imu.new_imu3!=0)
	{
		filter3.turning(imu.p3,imu.q3,imu.r3,imu.dt3);
	}
	else
	{
		filter3.blind_turning(imu.p3,imu.q3,imu.r3,imu.dt3);
	}

	init_filter.which_nb_to_keep(filter1.psi, filter2.psi, filter3.psi);
	if(init_filter.keep_nb == 1)
	{
		if (imu.new_imu1!=0)
		{
			init_filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		}
		else if (imu.new_imu2!=0)
		{
			init_filter.keep_nb = 2;
			init_filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		}
		else
		{
			init_filter.keep_nb = 3;
			init_filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		}
	}
	else if(init_filter.keep_nb == 2)
	{
		if (imu.new_imu2!=0)
		{
			init_filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		}
		else if (imu.new_imu1!=0)
		{
			init_filter.keep_nb = 1;
			init_filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		}
		else
		{
			init_filter.keep_nb = 3;
			init_filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		}
	}
	else
	{
		if (imu.new_imu3!=0)
		{
			init_filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		}
		else if (imu.new_imu1!=0)
		{
			init_filter.keep_nb = 1;
			init_filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		}
		else
		{
			init_filter.keep_nb = 2;
			init_filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		}
	}

	if (fabs(fmod(init_filter.psi-filter.E_north_angle,2*PI))<filter.north_angle_thresh || fabs(fmod(init_filter.psi-filter.E_north_angle,2*PI))>2*PI-filter.north_angle_thresh)
	{
		filter.north_angle = init_filter.psi;
		filter.P_north_angle = init_filter.P_psi;
	}
	else
	{	
		filter.north_angle = filter.E_north_angle;
		filter.P_north_angle = filter.north_angle_thresh*filter.north_angle_thresh;
	}

	imu.set_prev_counters();
	if(pause_switch==false) 
	{
		state = _run; 
		filter.initialize_states(0,0,3.14159265,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); 
		filter1.initialize_states(0,0,3.14159265,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); 
		filter2.initialize_states(0,0,3.14159265,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); 
		filter3.initialize_states(0,0,3.14159265,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); 
		filter.set_imu_offset(0,0); //x_offset, y_offset
		if (fabs(fmod(init_filter.psi-filter.E_north_angle,2*PI))<filter.north_angle_thresh || fabs(fmod(init_filter.psi-filter.E_north_angle,2*PI))>2*PI-filter.north_angle_thresh)
		{
			filter.north_angle = init_filter.psi;
			filter.P_north_angle = init_filter.P_psi;
		}
		else
		{	
			filter.north_angle = filter.E_north_angle;
			filter.P_north_angle = filter.north_angle_thresh*filter.north_angle_thresh;
		}
	}
	else state = _forklift_drive;
}

void NavigationFilter::run(User_Input_Nav_Act user_input_nav_act)
{
	encoders.adjustEncoderWrapError();
	encoders.calculateWheelDistancesFromEncoders();
	encoders.calculateDeltaDistance6Wheels(0, 0); //turnFlag, stopFlag
	imu.determine_new_data();
	imu.filter_imu_values();
	
	// Predict Methods
	if(pause_switch==false) 
	{
		if (turnFlag)
		{
			prev_stopped = false;
			collecting_accelerometer_data = false;
			collected_gyro_data = false;
			collected_gyro1_data = false;
			collected_gyro2_data = false;
			collected_gyro3_data = false;
			if (imu.new_imu1!=0)
			{
				filter1.turning(imu.p1,imu.q1,imu.r1,imu.dt1);
			}
			else
			{
				filter1.blind_turning(imu.p1,imu.q1,imu.r1,imu.dt1);
			}
			if (imu.new_imu2!=0)
			{
				filter2.turning(imu.p2,imu.q2,imu.r2,imu.dt2);
			}
			else
			{
				filter2.blind_turning(imu.p2,imu.q2,imu.r2,imu.dt2);
			}
			if (imu.new_imu3!=0)
			{
				filter3.turning(imu.p3,imu.q3,imu.r3,imu.dt3);
			}
			else
			{
				filter3.blind_turning(imu.p3,imu.q3,imu.r3,imu.dt3);
			}
		

			if(filter.keep_nb == 1)
			{
				filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			else if(filter.keep_nb == 2)
			{
				filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			else
			{
				filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}


			filter.clear_accelerometer_values();
			//got_sun_here = false;
			imu.clear_gyro1_values();
			imu.clear_gyro2_values();
			imu.clear_gyro3_values();
		}
		else if (stopFlag)
		{
			if (!prev_stopped)
			{
				filter.which_nb_to_keep(filter1.psi, filter2.psi, filter3.psi);
				if(filter.keep_nb == 1)
				{
					if (imu.new_imu1!=0)
					{
						filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					else if (imu.new_imu2!=0)
					{
						filter.keep_nb = 2;
						filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					else
					{
						filter.keep_nb = 3;
						filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
				}
				else if(filter.keep_nb == 2)
				{
					if (imu.new_imu2!=0)
					{
						filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					else if (imu.new_imu1!=0)
					{
						filter.keep_nb = 1;
						filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					else
					{
						filter.keep_nb = 3;
						filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
				}
				else
				{
					if (imu.new_imu3!=0)
					{
						filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					else if (imu.new_imu1!=0)
					{
						filter.keep_nb = 1;
						filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					else
					{
						filter.keep_nb = 2;
						filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
				}
				filter1.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
				filter2.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
				filter3.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y

			}


			if (!prev_stopped&&!collecting_accelerometer_data)
			{
				collecting_accelerometer_data = true;
			}
		
			if ((fabs(sqrt(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az)-1)< 0.05 && sqrt((imu.p)*(imu.p)+(imu.q)*(imu.q)+(imu.r)*(imu.r))<0.005) && encoders.delta_distance == 0)
			{
				if (collecting_accelerometer_data)
				{
					if (filter.ax_values.size() > 200)
					{
						collecting_accelerometer_data = false;
						filter.roll_pitch_G_update();
						filter.clear_accelerometer_values();
						filter1.initialize_states(filter.phi,filter.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
						filter2.initialize_states(filter.phi,filter.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
						filter3.initialize_states(filter.phi,filter.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					else
					{
						filter.collect_accelerometer_data(imu.ax, imu.ay, imu.az);
					}
				}
			}
			else
			{
				if (imu.new_imu1!=0)
				{
					filter1.dead_reckoning(imu.p1,imu.q1,imu.r1,encoders.delta_distance,imu.dt1);
				}
				else
				{
					filter1.blind_dead_reckoning(imu.p1,imu.q1,imu.r1,encoders.delta_distance,imu.dt1);
				}
				if (imu.new_imu2!=0)
				{
					filter2.dead_reckoning(imu.p2,imu.q2,imu.r2,encoders.delta_distance,imu.dt2);
				}
				else
				{
					filter2.blind_dead_reckoning(imu.p2,imu.q2,imu.r2,encoders.delta_distance,imu.dt2);
				}
				if (imu.new_imu3!=0)
				{
					filter3.dead_reckoning(imu.p3,imu.q3,imu.r3,encoders.delta_distance,imu.dt3);
				}
				else
				{
					filter3.blind_dead_reckoning(imu.p3,imu.q3,imu.r3,encoders.delta_distance,imu.dt3);
				}

				if(filter.keep_nb == 1)
				{
					filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
				}
				else if(filter.keep_nb == 2)
				{
					filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
				}
				else
				{
					filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
				}
			}
			prev_stopped = true;
		}
		else
		{
			prev_stopped = false;
			collecting_accelerometer_data = false;
			collected_gyro_data = false;
			collected_gyro1_data = false;
			collected_gyro2_data = false;
			collected_gyro3_data = false;
			filter.clear_accelerometer_values();
			//got_sun_here = false;
			imu.clear_gyro1_values();
			imu.clear_gyro2_values();
			imu.clear_gyro3_values();
			if (imu.new_imu1!=0)
			{
				filter1.dead_reckoning(imu.p1,imu.q1,imu.r1,encoders.delta_distance,imu.dt1);
			}
			else
			{
				filter1.blind_dead_reckoning(imu.p1,imu.q1,imu.r1,encoders.delta_distance,imu.dt1);
			}
			if (imu.new_imu2!=0)
			{
				filter2.dead_reckoning(imu.p2,imu.q2,imu.r2,encoders.delta_distance,imu.dt2);
			}
			else
			{
				filter2.blind_dead_reckoning(imu.p2,imu.q2,imu.r2,encoders.delta_distance,imu.dt2);
			}
			if (imu.new_imu3!=0)
			{
				filter3.dead_reckoning(imu.p3,imu.q3,imu.r3,encoders.delta_distance,imu.dt3);
			}
			else
			{
				filter3.blind_dead_reckoning(imu.p3,imu.q3,imu.r3,encoders.delta_distance,imu.dt3);
			}
		

			if(filter.keep_nb == 1)
			{
				filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			else if(filter.keep_nb == 2)
			{
				filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			else
			{
				filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}

	}
	else 
	{
		prev_stopped = false;
		collecting_accelerometer_data = false;
		collected_gyro_data = false;
		collected_gyro1_data = false;
		collected_gyro2_data = false;
		collected_gyro3_data = false;
		filter.clear_accelerometer_values();
		imu.clear_gyro1_values();
		imu.clear_gyro2_values();
		imu.clear_gyro3_values();
		//got_sun_here = false;
		if ((fabs(sqrt(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az)-1)< 0.05 && sqrt((imu.p)*(imu.p)+(imu.q)*(imu.q)+(imu.r)*(imu.r))<0.01) && encoders.delta_distance == 0)
		{
		}
		else
		{
			if (imu.new_imu1!=0)
			{
				filter1.dead_reckoning(imu.p1,imu.q1,imu.r1,encoders.delta_distance,imu.dt1);
			}
			else
			{
				filter1.blind_dead_reckoning(imu.p1,imu.q1,imu.r1,encoders.delta_distance,imu.dt1);
			}
			if (imu.new_imu2!=0)
			{
				filter2.dead_reckoning(imu.p2,imu.q2,imu.r2,encoders.delta_distance,imu.dt2);
			}
			else
			{
				filter2.blind_dead_reckoning(imu.p2,imu.q2,imu.r2,encoders.delta_distance,imu.dt2);
			}
			if (imu.new_imu3!=0)
			{
				filter3.dead_reckoning(imu.p3,imu.q3,imu.r3,encoders.delta_distance,imu.dt3);
			}
			else
			{
				filter3.blind_dead_reckoning(imu.p3,imu.q3,imu.r3,encoders.delta_distance,imu.dt3);
			}
			
			if(filter.keep_nb == 1)
			{
				filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			else if(filter.keep_nb == 2)
			{
				filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			else
			{
				filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}
	}
}

void NavigationFilter::getExecInfoCallback(const messages::ExecInfo::ConstPtr &msg)
{
	this->pause_switch = msg->pause;
	this->turnFlag = 0;
	this->stopFlag = 0;
}