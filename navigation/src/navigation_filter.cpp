#include <navigation/navigation_filter.hpp>

NavigationFilter::NavigationFilter()
{ 
	sub_exec = nh.subscribe("/control/exec/info", 1, &NavigationFilter::getExecInfoCallback, this);
	pause_switch = false;
	stopFlag = false;
	turnFlag = false;

	sub_mission = nh.subscribe("/control/missionplanning/info", 1, &NavigationFilter::getMissionPlanningInfoCallback, this);

	sub_lidar = nh.subscribe("lidar/lidarfilteringout/lidarfilteringout", 1, &NavigationFilter::getLidarFilterOutCallback, this);
    homing_x=0;
	homing_y=0;
	homing_heading=0;
    homing_found=false;

	filter.initialize_states(0,0,0,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y);
	init_filter.initialize_states(0,0,0,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y);
	filter1.initialize_states(0,0,0,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y);
	filter2.initialize_states(0,0,0,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y);
	filterS.initialize_states(0,0,0,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y);

	encoders.set_wheel_radius(0.2286/2);
	encoders.set_counts_per_revolution(4476.16*1.062);
	current_time = ros::Time::now().toSec();

    //added for new User Interface -Matt G.
    std::string tempServiceName = "";
    if(!(ros::param::get("NavControlServiceName", tempServiceName)))
    {
        //if the parameter does not exist, use this default one
        tempServiceName = "/navigation/navigationfilter/control";
    }
    nav_control_server = nh.advertiseService(tempServiceName,
                                                &NavigationFilter::navFilterControlServiceCallback,
                                                    this);

}

void NavigationFilter::update_time()
{
	dt = ros::Time::now().toSec() - current_time;
	current_time = ros::Time::now().toSec();
}

void NavigationFilter::waiting(User_Input_Nav_Act user_input_nav_act)
{
    if (user_input_nav_act.bias_removal_forklift!=1 && !(latest_nav_control_request.runBiasRemoval))
	{
		first_pass = true;
		//ROS_INFO("user_input_nav_act.bias_removal_forklift!=1");
	}
	else
	{
		//ROS_INFO("user_input_nav_act.bias_removal_forklift==1");
	}
	if (first_pass == true)
	{
		init_filter.clear_accelerometer_values();
		imu.clear_gyro_values();
		prev_stopped = false;
		collecting_accelerometer_data = false;
		collected_gyro_data = false;
		collected_gyro1_data = false;
		collected_gyro2_data = false;
		collected_gyro3_data = false;
		collected_gyro4_data = false;
		collected_gyro5_data = false;
		collected_gyro6_data = false;
		collected_gyroS_data = false;
		first_pass = false;
	}

	if ((fabs(sqrt(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az)-1)< 0.05 && sqrt((imu.p)*(imu.p)+(imu.q)*(imu.q)+(imu.r)*(imu.r))<0.01) && encoders.delta_distance == 0)
	{
		if (collecting_accelerometer_data)
		{
			if (init_filter.ax_values.size() > 200)
			{
				collecting_accelerometer_data = false;
				init_filter.roll_pitch_G_update();
				init_filter.clear_accelerometer_values();
			}
			else
			{
				if(imu.new_nb1+imu.new_nb2!=0)
				{
					init_filter.collect_accelerometer_data(imu.ax, imu.ay, imu.az);
				}
			}
		}
        if (user_input_nav_act.bias_removal_forklift==1 || latest_nav_control_request.runBiasRemoval)
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

			if (imu.p4_values.size() > 500 && collected_gyro4_data!=true)
			{
				collected_gyro4_data = true;
				imu.calculate_gyro4_offset();
				imu.set_gyro4_offset();
			}
			else if (collected_gyro4_data==true)
			{
				collected_gyro4_data = true;
			}
			else
			{
				imu.collect_gyro4_data();
				collected_gyro4_data = false;
			}

			if (imu.p5_values.size() > 500 && collected_gyro5_data!=true)
			{
				collected_gyro5_data = true;
				imu.calculate_gyro5_offset();
				imu.set_gyro5_offset();
			}
			else if (collected_gyro5_data==true)
			{
				collected_gyro5_data = true;
			}
			else
			{
				imu.collect_gyro5_data();
				collected_gyro5_data = false;
			}

			if (imu.p6_values.size() > 500 && collected_gyro6_data!=true)
			{
				collected_gyro6_data = true;
				imu.calculate_gyro6_offset();
				imu.set_gyro6_offset();
			}
			else if (collected_gyro6_data==true)
			{
				collected_gyro6_data = true;
			}
			else
			{
				imu.collect_gyro6_data();
                collected_gyro6_data = false;
			}

			if (imu.pS_values.size() > 500 && collected_gyroS_data!=true)
			{
				collected_gyroS_data = true;
				imu.calculate_gyroS_offset();
				imu.set_gyroS_offset();
			}
			else if (collected_gyroS_data==true)
			{
				collected_gyroS_data = true;
			}
			else
			{
				imu.collect_gyroS_data();
                collected_gyroS_data = false;
			}

			if (collected_gyro1_data && collected_gyro2_data && collected_gyro3_data && collected_gyro4_data && collected_gyro5_data && collected_gyro6_data && collected_gyroS_data)
			{
				collected_gyro_data = true;
			}
			else
			{
				collected_gyro_data = false;
			}
		}
	}
    if ((user_input_nav_act.bias_removal_forklift==1 || latest_nav_control_request.runBiasRemoval) && collected_gyro_data)
	{
		nav_status_output = 1;
	}
	else
	{
		nav_status_output = 0;
	}
    //ROS_INFO("collected_gyro_data = %d \n", collected_gyro_data);

    if(user_input_nav_act.begin_dead_reckoning==1 || latest_nav_control_request.beginForkliftDeadReckoning)
	{
		state = _forklift_drive;
		ROS_INFO("init north angle = %f",init_north_angle*RAD_2_DEG);
		init_filter.initialize_states(init_filter.phi,init_filter.theta,init_north_angle,1,0,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		filter1.initialize_states(init_filter.phi,init_filter.theta,init_north_angle,1,0,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		filter2.initialize_states(init_filter.phi,init_filter.theta,init_north_angle,1,0,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		filterS.initialize_states(init_filter.phi,init_filter.theta,init_north_angle,1,0,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		init_filter.initialize_variance(collected_gyro_data,2.0*PI/180.0); //performed bias removal, north_angle_unc
		first_pass = true;
	}
	else state = _waiting;
	calibrate_counter++;
}

void NavigationFilter::forklift_drive(User_Input_Nav_Act user_input_nav_act)
{
    init_filter.which_nb_to_keep(imu.nb1_drive_counter, imu.nb1_current, imu.nb1_good, imu.nb1_good_prev, imu.nb2_drive_counter, imu.nb2_current, imu.nb2_good, imu.nb2_good_prev, imu.nbS_current, imu.nbS_good, imu.nbS_good_prev);
	if (imu.new_nb1!=0)
	{
		filter1.turning(imu.nb1_p,imu.nb1_q,imu.nb1_r,imu.dt1);
	}
	else
	{
		filter1.blind_turning(imu.nb1_p,imu.nb1_q,imu.nb1_r,imu.dt1);
	}

	if (imu.new_nb2!=0)
	{
		filter2.turning(imu.nb2_p,imu.nb2_q,imu.nb2_r,imu.dt2);
	}
	else
	{
		filter2.blind_turning(imu.nb2_p,imu.nb2_q,imu.nb2_r,imu.dt2);
	}

	if (imu.new_nbS!=0)
	{
		filterS.turning(imu.nbS_p,imu.nbS_q,imu.nbS_r,imu.dtS);
	}
	else
	{
		filterS.blind_turning(imu.nbS_p,imu.nbS_q,imu.nbS_r,imu.dtS);
	}


	if(init_filter.keep_nb == 3 || init_filter.keep_nb == 28 || init_filter.keep_nb == 1 || init_filter.keep_nb == 5 || init_filter.keep_nb == 7 || init_filter.keep_nb == 9 || init_filter.keep_nb == 11 || init_filter.keep_nb == 12 || init_filter.keep_nb == 14 || init_filter.keep_nb == 20 || init_filter.keep_nb == 22 || init_filter.keep_nb == 24 || init_filter.keep_nb == 26 || init_filter.keep_nb == 29 || init_filter.keep_nb == 31 || init_filter.keep_nb == 33 || init_filter.keep_nb == 35)
	{
		init_filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
	}
	else if (init_filter.keep_nb == 2 || init_filter.keep_nb == 6 || init_filter.keep_nb == 8 || init_filter.keep_nb == 10 || init_filter.keep_nb == 15 || init_filter.keep_nb == 16 || init_filter.keep_nb == 18 || init_filter.keep_nb == 21 || init_filter.keep_nb == 23 || init_filter.keep_nb == 25 || init_filter.keep_nb == 27 || init_filter.keep_nb == 30 || init_filter.keep_nb == 32 || init_filter.keep_nb == 34 || init_filter.keep_nb == 36)
	{
		init_filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
	}
	else if (init_filter.keep_nb == 4 || init_filter.keep_nb == 13 || init_filter.keep_nb == 17 || init_filter.keep_nb == 19)
	{
		init_filter.initialize_states(filterS.phi,filterS.theta,filterS.psi,filterS.x,filterS.y,filterS.P_phi,filterS.P_theta,filterS.P_psi,filterS.P_x,filterS.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
	}


	if ((imu.nb1_missed_counter>50 && imu.nb1_good || imu.nb2_missed_counter>50 && imu.nb2_good) && stop_request == false)
	{
		stop_request = true;
		stop_time = ros::Time::now().toSec();
	}
    else if (stop_request == true && ((imu.nb1_current && imu.nb2_current)||(imu.nb1_current && !imu.nb2_good)||(!imu.nb1_good && imu.nb2_current)))
	{
		stop_request = false;

		if(imu.nb1_current)
		{
			imu.nb1_good_prev = true;
		}
		if(imu.nb2_current)
		{
			imu.nb2_good_prev = true;
		}
		if(imu.nbS_current)
		{
			imu.nbS_good_prev = true;
		}

		if(init_filter.keep_nb == 1 || init_filter.keep_nb == 5 || init_filter.keep_nb == 7 || init_filter.keep_nb == 9 || init_filter.keep_nb == 11 || init_filter.keep_nb == 12 || init_filter.keep_nb == 14 || init_filter.keep_nb == 20 || init_filter.keep_nb == 22 || init_filter.keep_nb == 24 || init_filter.keep_nb == 26 || init_filter.keep_nb == 29 || init_filter.keep_nb == 31 || init_filter.keep_nb == 33 || init_filter.keep_nb == 35)
		{
			init_filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			if(init_filter.keep_nb == 1 || init_filter.keep_nb == 5 || init_filter.keep_nb == 7 || init_filter.keep_nb == 9)
			{
				filter2.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			if(init_filter.keep_nb == 1 || init_filter.keep_nb == 5 || init_filter.keep_nb == 7 || init_filter.keep_nb == 9 || init_filter.keep_nb == 11 || init_filter.keep_nb == 14)
			{
				filterS.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}
		else if (init_filter.keep_nb == 2 || init_filter.keep_nb == 6 || init_filter.keep_nb == 8 || init_filter.keep_nb == 10 || init_filter.keep_nb == 15 || init_filter.keep_nb == 16 || init_filter.keep_nb == 18 || init_filter.keep_nb == 21 || init_filter.keep_nb == 23 || init_filter.keep_nb == 25 || init_filter.keep_nb == 27 || init_filter.keep_nb == 30 || init_filter.keep_nb == 32 || init_filter.keep_nb == 34 || init_filter.keep_nb == 36)
		{

			init_filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			if(init_filter.keep_nb == 2 || init_filter.keep_nb == 6 || init_filter.keep_nb == 8 || init_filter.keep_nb == 10)
			{
				filter1.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			if(init_filter.keep_nb == 2 || init_filter.keep_nb == 6 || init_filter.keep_nb == 8 || init_filter.keep_nb == 10 || init_filter.keep_nb == 15 || init_filter.keep_nb == 18)
			{
				filterS.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}
		else if (init_filter.keep_nb == 3 || init_filter.keep_nb == 28)
		{
			init_filter.initialize_states((filter1.phi+filter2.phi)/2.0,(filter1.theta+filter2.theta)/2.0,(filter1.psi+filter2.psi)/2.0,(filter1.x+filter2.x)/2.0,(filter1.y+filter2.y)/2.0,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			filter1.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			filter2.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			if (filter.keep_nb == 3)
			{
				filterS.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}
		else if (init_filter.keep_nb == 4 || init_filter.keep_nb == 13 || init_filter.keep_nb == 17 || init_filter.keep_nb == 19)
		{
			init_filter.initialize_states(filterS.phi,filterS.theta,filterS.psi,filterS.x,filterS.y,filterS.P_phi,filterS.P_theta,filterS.P_psi,filterS.P_x,filterS.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			if (init_filter.keep_nb == 4 || init_filter.keep_nb == 13)
			{
				filter1.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			if (init_filter.keep_nb == 4 || init_filter.keep_nb == 17)
			{
				filter2.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}
	}
    else if (ros::Time::now().toSec()-stop_time>20.0)
	{
		stop_request = false;
        if (imu.nb1_missed_counter>50)
		{
			 imu.nb1_good = false;
			 imu.nb1_good_prev = false;
		}
        if (imu.nb2_missed_counter>50)
		{
			 imu.nb2_good = false;
			 imu.nb2_good_prev = false;
		}
		if (imu.nbS_missed_counter>50)
		{
			 imu.nbS_good = false;
			 imu.nbS_good_prev = false;
		}
	}


	if(pause_switch==false && !stop_request) 
	{
		if(imu.nb1_current)
		{
			imu.nb1_good_prev = true;
		}
		if(imu.nb2_current)
		{
			imu.nb2_good_prev = true;
		}
		if(imu.nbS_current)
		{
			imu.nbS_good_prev = true;
		}

		if(init_filter.keep_nb == 1 || init_filter.keep_nb == 5 || init_filter.keep_nb == 7 || init_filter.keep_nb == 9 || init_filter.keep_nb == 11 || init_filter.keep_nb == 12 || init_filter.keep_nb == 14 || init_filter.keep_nb == 20 || init_filter.keep_nb == 22 || init_filter.keep_nb == 24 || init_filter.keep_nb == 26 || init_filter.keep_nb == 29 || init_filter.keep_nb == 31 || init_filter.keep_nb == 33 || init_filter.keep_nb == 35)
		{
			init_filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			if(init_filter.keep_nb == 1 || init_filter.keep_nb == 5 || init_filter.keep_nb == 7 || init_filter.keep_nb == 9)
			{
				filter2.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			if(init_filter.keep_nb == 1 || init_filter.keep_nb == 5 || init_filter.keep_nb == 7 || init_filter.keep_nb == 9 || init_filter.keep_nb == 11 || init_filter.keep_nb == 14)
			{
				filterS.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}
		else if (init_filter.keep_nb == 2 || init_filter.keep_nb == 6 || init_filter.keep_nb == 8 || init_filter.keep_nb == 10 || init_filter.keep_nb == 15 || init_filter.keep_nb == 16 || init_filter.keep_nb == 18 || init_filter.keep_nb == 21 || init_filter.keep_nb == 23 || init_filter.keep_nb == 25 || init_filter.keep_nb == 27 || init_filter.keep_nb == 30 || init_filter.keep_nb == 32 || init_filter.keep_nb == 34 || init_filter.keep_nb == 36)
		{

			init_filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			if(init_filter.keep_nb == 2 || init_filter.keep_nb == 6 || init_filter.keep_nb == 8 || init_filter.keep_nb == 10)
			{
				filter1.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			if(init_filter.keep_nb == 2 || init_filter.keep_nb == 6 || init_filter.keep_nb == 8 || init_filter.keep_nb == 10 || init_filter.keep_nb == 15 || init_filter.keep_nb == 18)
			{
				filterS.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}
		else if (init_filter.keep_nb == 3 || init_filter.keep_nb == 28)
		{
			init_filter.initialize_states((filter1.phi+filter2.phi)/2.0,(filter1.theta+filter2.theta)/2.0,(filter1.psi+filter2.psi)/2.0,(filter1.x+filter2.x)/2.0,(filter1.y+filter2.y)/2.0,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			filter1.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			filter2.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			if (filter.keep_nb == 3)
			{
				filterS.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}
		else if (init_filter.keep_nb == 4 || init_filter.keep_nb == 13 || init_filter.keep_nb == 17 || init_filter.keep_nb == 19)
		{
			init_filter.initialize_states(filterS.phi,filterS.theta,filterS.psi,filterS.x,filterS.y,filterS.P_phi,filterS.P_theta,filterS.P_psi,filterS.P_x,filterS.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			if (init_filter.keep_nb == 4 || init_filter.keep_nb == 13)
			{
				filter1.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			if (init_filter.keep_nb == 4 || init_filter.keep_nb == 17)
			{
				filter2.initialize_states(init_filter.phi,init_filter.theta,init_filter.psi,init_filter.x,init_filter.y,init_filter.P_phi,init_filter.P_theta,init_filter.P_psi,init_filter.P_x,init_filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}

        /*if (fabs(fmod(init_filter.psi-filter.E_north_angle,2*PI))<filter.north_angle_thresh || fabs(fmod(init_filter.psi-filter.E_north_angle,2*PI))>2*PI-filter.north_angle_thresh)
        {
            filter.north_angle = init_filter.psi;
            filter.P_north_angle = init_filter.P_psi;
        }
        else
        {
            filter.north_angle = filter.E_north_angle;
            filter.P_north_angle = filter.north_angle_thresh*filter.north_angle_thresh;
        }*/

        filter.north_angle = init_filter.psi;
        ROS_INFO("init_filter.psi (north angle = %f)",init_filter.psi);
		filter.P_north_angle = init_filter.P_psi;

		state = _run; 
		filter.initialize_states(0,0,0,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); 
		filter1.initialize_states(0,0,0,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); 
		filter2.initialize_states(0,0,0,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y);
		filterS.initialize_states(0,0,0,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); 
	}
	else state = _forklift_drive;
}

void NavigationFilter::run(User_Input_Nav_Act user_input_nav_act)
{
    filter.which_nb_to_keep(imu.nb1_drive_counter, imu.nb1_current, imu.nb1_good, imu.nb1_good_prev, imu.nb2_drive_counter, imu.nb2_current, imu.nb2_good, imu.nb2_good_prev, imu.nbS_current, imu.nbS_good, imu.nbS_good_prev);
    do_homing = false;

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
			collected_gyro4_data = false;
			collected_gyro5_data = false;
			collected_gyro6_data = false;
			collected_gyroS_data = false;

			if (imu.new_nb1!=0)
			{
				filter1.turning(imu.nb1_p,imu.nb1_q,imu.nb1_r,imu.dt1);
			}
			else
			{
				filter1.blind_turning(imu.nb1_p,imu.nb1_q,imu.nb1_r,imu.dt1);
			}

			if (imu.new_nb2!=0)
			{
				filter2.turning(imu.nb2_p,imu.nb2_q,imu.nb2_r,imu.dt2);
			}
			else
			{
				filter2.blind_turning(imu.nb2_p,imu.nb2_q,imu.nb2_r,imu.dt2);
			}

			if (imu.new_nbS!=0)
			{
				filterS.turning(imu.nbS_p,imu.nbS_q,imu.nbS_r,imu.dtS);
			}
			else
			{
				filterS.blind_turning(imu.nbS_p,imu.nbS_q,imu.nbS_r,imu.dtS);
			}


			if(filter.keep_nb == 3 || filter.keep_nb == 28 || filter.keep_nb == 1 || filter.keep_nb == 5 || filter.keep_nb == 7 || filter.keep_nb == 9 || filter.keep_nb == 11 || filter.keep_nb == 12 || filter.keep_nb == 14 || filter.keep_nb == 20 || filter.keep_nb == 22 || filter.keep_nb == 24 || filter.keep_nb == 26 || filter.keep_nb == 29 || filter.keep_nb == 31 || filter.keep_nb == 33 || filter.keep_nb == 35)
			{
				filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			else if (filter.keep_nb == 2 || filter.keep_nb == 6 || filter.keep_nb == 8 || filter.keep_nb == 10 || filter.keep_nb == 15 || filter.keep_nb == 16 || filter.keep_nb == 18 || filter.keep_nb == 21 || filter.keep_nb == 23 || filter.keep_nb == 25 || filter.keep_nb == 27 || filter.keep_nb == 30 || filter.keep_nb == 32 || filter.keep_nb == 34 || filter.keep_nb == 36)
			{

				filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			else if (filter.keep_nb == 4 || filter.keep_nb == 13 || filter.keep_nb == 17 || filter.keep_nb == 19)
			{
				filter.initialize_states(filterS.phi,filterS.theta,filterS.psi,filterS.x,filterS.y,filterS.P_phi,filterS.P_theta,filterS.P_psi,filterS.P_x,filterS.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}


			filter.clear_accelerometer_values();
			imu.clear_gyro_values();
		}
		else if (stopFlag)
		{
			if (!prev_stopped && !stop_request)
			{
				if(imu.nb1_current)
				{
					imu.nb1_good_prev = true;
				}
				if(imu.nb2_current)
				{
					imu.nb2_good_prev = true;
				}
				if(imu.nbS_current)
				{
					imu.nbS_good_prev = true;
				}

				if(filter.keep_nb == 1 || filter.keep_nb == 5 || filter.keep_nb == 7 || filter.keep_nb == 9 || filter.keep_nb == 11 || filter.keep_nb == 12 || filter.keep_nb == 14 || filter.keep_nb == 20 || filter.keep_nb == 22 || filter.keep_nb == 24 || filter.keep_nb == 26 || filter.keep_nb == 29 || filter.keep_nb == 31 || filter.keep_nb == 33 || filter.keep_nb == 35)
				{
					filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					if(filter.keep_nb == 1 || filter.keep_nb == 5 || filter.keep_nb == 7 || filter.keep_nb == 9)
					{
						filter2.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					if(filter.keep_nb == 1 || filter.keep_nb == 5 || filter.keep_nb == 7 || filter.keep_nb == 9 || filter.keep_nb == 11 || filter.keep_nb == 14)
					{
						filterS.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
				}
				else if (filter.keep_nb == 2 || filter.keep_nb == 6 || filter.keep_nb == 8 || filter.keep_nb == 10 || filter.keep_nb == 15 || filter.keep_nb == 16 || filter.keep_nb == 18 || filter.keep_nb == 21 || filter.keep_nb == 23 || filter.keep_nb == 25 || filter.keep_nb == 27 || filter.keep_nb == 30 || filter.keep_nb == 32 || filter.keep_nb == 34 || filter.keep_nb == 36)
				{

					filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					if(filter.keep_nb == 2 || filter.keep_nb == 6 || filter.keep_nb == 8 || filter.keep_nb == 10)
					{
						filter1.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					if(filter.keep_nb == 2 || filter.keep_nb == 6 || filter.keep_nb == 8 || filter.keep_nb == 10 || filter.keep_nb == 15 || filter.keep_nb == 18)
					{
						filterS.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
				}
				else if (filter.keep_nb == 3 || filter.keep_nb == 28)
				{
					filter.initialize_states((filter1.phi+filter2.phi)/2.0,(filter1.theta+filter2.theta)/2.0,(filter1.psi+filter2.psi)/2.0,(filter1.x+filter2.x)/2.0,(filter1.y+filter2.y)/2.0,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					filter1.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					filter2.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					if (filter.keep_nb == 3)
					{
						filterS.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
				}
				else if (filter.keep_nb == 4 || filter.keep_nb == 13 || filter.keep_nb == 17 || filter.keep_nb == 19)
				{
					filter.initialize_states(filterS.phi,filterS.theta,filterS.psi,filterS.x,filterS.y,filterS.P_phi,filterS.P_theta,filterS.P_psi,filterS.P_x,filterS.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					if (filter.keep_nb == 4 || filter.keep_nb == 13)
					{
						filter1.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					if (filter.keep_nb == 4 || filter.keep_nb == 17)
					{
						filter2.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
				}
			}


			if (!prev_stopped&&!collecting_accelerometer_data)
			{
				collecting_accelerometer_data = true;
			}

			if ((fabs(sqrt(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az)-1)< 0.05 && sqrt((imu.p)*(imu.p)+(imu.q)*(imu.q)+(imu.r)*(imu.r))<0.05) && encoders.delta_distance < 5)
			{
				if(sqrt(filter.x*filter.x+filter.y*filter.y)<30.0 || possibly_lost)
				{
					do_homing = true;
				}
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
						filterS.initialize_states(filter.phi,filter.theta,filterS.psi,filterS.x,filterS.y,filterS.P_phi,filterS.P_theta,filterS.P_psi,filterS.P_x,filterS.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					else
					{
						if(imu.new_nb1+imu.new_nb2!=0)
						{
							filter.collect_accelerometer_data(imu.ax, imu.ay, imu.az);
						}
					}
				}
				if (latest_nav_control_request.runBiasRemoval)
				{
					if (imu.p1_values.size() > 500 && collected_gyro1_data!=true)
					{
						imu.calculate_gyro1_offset();
						if(imu.good_bias1)
						{
							collected_gyro1_data = true;
							imu.set_gyro1_offset();
							filter1.Q_phi = 2.2847e-008;
							filter1.Q_theta = 2.2847e-008;
							filter1.Q_psi = 2.2847e-008;
						}
						else
						{
							imu.clear_gyro1_values();
						}
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
						imu.calculate_gyro2_offset();
						if(imu.good_bias2)
						{
							collected_gyro2_data = true;
							imu.set_gyro2_offset();
							filter1.Q_phi = 2.2847e-008;
							filter1.Q_theta = 2.2847e-008;
							filter1.Q_psi = 2.2847e-008;
						}
						else
						{
							imu.clear_gyro2_values();
						}
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
						imu.calculate_gyro3_offset();
						if(imu.good_bias3)
						{
							collected_gyro3_data = true;
							imu.set_gyro3_offset();
							filter1.Q_phi = 2.2847e-008;
							filter1.Q_theta = 2.2847e-008;
							filter1.Q_psi = 2.2847e-008;
						}
						else
						{
							imu.clear_gyro3_values();
						}
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

					if (imu.p4_values.size() > 500 && collected_gyro4_data!=true)
					{
						imu.calculate_gyro4_offset();
						if(imu.good_bias4)
						{
							collected_gyro4_data = true;
							imu.set_gyro4_offset();
							filter2.Q_phi = 2.2847e-008;
							filter2.Q_theta = 2.2847e-008;
							filter2.Q_psi = 2.2847e-008;
						}
						else
						{
							imu.clear_gyro4_values();
						}
					}
					else if (collected_gyro4_data==true)
					{
						collected_gyro4_data = true;
					}
					else
					{
						imu.collect_gyro4_data();
						collected_gyro4_data = false;
					}

					if (imu.p5_values.size() > 500 && collected_gyro5_data!=true)
					{
						imu.calculate_gyro5_offset();
						if(imu.good_bias5)
						{
							collected_gyro5_data = true;
							imu.set_gyro5_offset();
							filter2.Q_phi = 2.2847e-008;
							filter2.Q_theta = 2.2847e-008;
							filter2.Q_psi = 2.2847e-008;
						}
						else
						{
							imu.clear_gyro5_values();
						}
					}
					else if (collected_gyro5_data==true)
					{
						collected_gyro5_data = true;
					}
					else
					{
						imu.collect_gyro5_data();
						collected_gyro5_data = false;
					}

					if (imu.p6_values.size() > 500 && collected_gyro6_data!=true)
					{
						imu.calculate_gyro6_offset();
						if(imu.good_bias6)
						{
							collected_gyro6_data = true;
							imu.set_gyro6_offset();
							filter2.Q_phi = 2.2847e-008;
							filter2.Q_theta = 2.2847e-008;
							filter2.Q_psi = 2.2847e-008;
						}
						else
						{
							imu.clear_gyro6_values();
						}
					}
					else if (collected_gyro6_data==true)
					{
						collected_gyro6_data = true;
					}
					else
					{
						imu.collect_gyro6_data();
						collected_gyro6_data = false;
					}

					if (imu.pS_values.size() > 500 && collected_gyroS_data!=true)
					{
						imu.calculate_gyroS_offset();
						if(imu.good_biasS)
						{
							collected_gyroS_data = true;
							imu.set_gyroS_offset();
							filterS.Q_phi = 2.2847e-008;
							filterS.Q_theta = 2.2847e-008;
							filterS.Q_psi = 2.2847e-008;
						}
						else
						{
							imu.clear_gyroS_values();
						}
					}
					else if (collected_gyroS_data==true)
					{
						collected_gyroS_data = true;
					}
					else
					{
						imu.collect_gyroS_data();
						collected_gyroS_data = false;
					}


					if (collected_gyro1_data && collected_gyro2_data && collected_gyro3_data && collected_gyro4_data && collected_gyro5_data && collected_gyro6_data && collected_gyroS_data)
					{
						collected_gyro_data = true;
					}
					else
					{
						collected_gyro_data = false;
					}
				}
			}
			else
			{
				if (imu.new_nb1!=0)
				{
					filter1.dead_reckoning(imu.nb1_p,imu.nb1_q,imu.nb1_r,encoders.delta_distance,imu.dt1);
				}
				else
				{
					filter1.blind_dead_reckoning(imu.nb1_p,imu.nb1_q,imu.nb1_r,encoders.delta_distance,imu.dt1);
				}

				if (imu.new_nb2!=0)
				{
					filter2.dead_reckoning(imu.nb2_p,imu.nb2_q,imu.nb2_r,encoders.delta_distance,imu.dt2);
				}
				else
				{
					filter2.blind_dead_reckoning(imu.nb2_p,imu.nb2_q,imu.nb2_r,encoders.delta_distance,imu.dt2);
				}

				if (imu.new_nbS!=0)
				{
					filterS.dead_reckoning(imu.nbS_p,imu.nbS_q,imu.nbS_r,encoders.delta_distance,imu.dtS);
				}
				else
				{
					filterS.blind_dead_reckoning(imu.nbS_p,imu.nbS_q,imu.nbS_r,encoders.delta_distance,imu.dtS);
				}



				if(filter.keep_nb == 3 || filter.keep_nb == 28 || filter.keep_nb == 1 || filter.keep_nb == 5 || filter.keep_nb == 7 || filter.keep_nb == 9 || filter.keep_nb == 11 || filter.keep_nb == 12 || filter.keep_nb == 14 || filter.keep_nb == 20 || filter.keep_nb == 22 || filter.keep_nb == 24 || filter.keep_nb == 26 || filter.keep_nb == 29 || filter.keep_nb == 31 || filter.keep_nb == 33 || filter.keep_nb == 35)
				{
					filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
				}
				else if (filter.keep_nb == 2 || filter.keep_nb == 6 || filter.keep_nb == 8 || filter.keep_nb == 10 || filter.keep_nb == 15 || filter.keep_nb == 16 || filter.keep_nb == 18 || filter.keep_nb == 21 || filter.keep_nb == 23 || filter.keep_nb == 25 || filter.keep_nb == 27 || filter.keep_nb == 30 || filter.keep_nb == 32 || filter.keep_nb == 34 || filter.keep_nb == 36)
				{

					filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
				}
				else if (filter.keep_nb == 4 || filter.keep_nb == 13 || filter.keep_nb == 17 || filter.keep_nb == 19)
				{
					filter.initialize_states(filterS.phi,filterS.theta,filterS.psi,filterS.x,filterS.y,filterS.P_phi,filterS.P_theta,filterS.P_psi,filterS.P_x,filterS.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
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
			collected_gyro4_data = false;
			collected_gyro5_data = false;
			collected_gyro6_data = false;
			collected_gyroS_data = false;
			filter.clear_accelerometer_values();
			imu.clear_gyro_values();
			if (imu.new_nb1!=0)
			{
				filter1.dead_reckoning(imu.nb1_p,imu.nb1_q,imu.nb1_r,encoders.delta_distance,imu.dt1);
			}
			else
			{
				filter1.blind_dead_reckoning(imu.nb1_p,imu.nb1_q,imu.nb1_r,encoders.delta_distance,imu.dt1);
			}
			if (imu.new_nb2!=0)
			{
				filter2.dead_reckoning(imu.nb2_p,imu.nb2_q,imu.nb2_r,encoders.delta_distance,imu.dt2);
			}
			else
			{
				filter2.blind_dead_reckoning(imu.nb2_p,imu.nb2_q,imu.nb2_r,encoders.delta_distance,imu.dt2);
			}

			if (imu.new_nbS!=0)
			{
				filterS.dead_reckoning(imu.nbS_p,imu.nbS_q,imu.nbS_r,encoders.delta_distance,imu.dtS);
			}
			else
			{
				filterS.blind_dead_reckoning(imu.nbS_p,imu.nbS_q,imu.nbS_r,encoders.delta_distance,imu.dtS);
			}



		

			if(filter.keep_nb == 3 || filter.keep_nb == 28 || filter.keep_nb == 1 || filter.keep_nb == 5 || filter.keep_nb == 7 || filter.keep_nb == 9 || filter.keep_nb == 11 || filter.keep_nb == 12 || filter.keep_nb == 14 || filter.keep_nb == 20 || filter.keep_nb == 22 || filter.keep_nb == 24 || filter.keep_nb == 26 || filter.keep_nb == 29 || filter.keep_nb == 31 || filter.keep_nb == 33 || filter.keep_nb == 35)
			{
				filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			else if (filter.keep_nb == 2 || filter.keep_nb == 6 || filter.keep_nb == 8 || filter.keep_nb == 10 || filter.keep_nb == 15 || filter.keep_nb == 16 || filter.keep_nb == 18 || filter.keep_nb == 21 || filter.keep_nb == 23 || filter.keep_nb == 25 || filter.keep_nb == 27 || filter.keep_nb == 30 || filter.keep_nb == 32 || filter.keep_nb == 34 || filter.keep_nb == 36)
			{

				filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			else if (filter.keep_nb == 4 || filter.keep_nb == 13 || filter.keep_nb == 17 || filter.keep_nb == 19)
			{
				filter.initialize_states(filterS.phi,filterS.theta,filterS.psi,filterS.x,filterS.y,filterS.P_phi,filterS.P_theta,filterS.P_psi,filterS.P_x,filterS.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
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
		collected_gyro4_data = false;
		collected_gyro5_data = false;
		collected_gyro6_data = false;
		collected_gyroS_data = false;
		filter.clear_accelerometer_values();
		imu.clear_gyro_values();
		if ((fabs(sqrt(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az)-1)< 0.05 && sqrt((imu.p)*(imu.p)+(imu.q)*(imu.q)+(imu.r)*(imu.r))<0.01) && encoders.delta_distance == 0)
		{
		}
		else
		{
			if (imu.new_nb1!=0)
			{
				filter1.dead_reckoning(imu.nb1_p,imu.nb1_q,imu.nb1_r,encoders.delta_distance,imu.dt1);
			}
			else
			{
				filter1.blind_dead_reckoning(imu.nb1_p,imu.nb1_q,imu.nb1_r,encoders.delta_distance,imu.dt1);
			}

			if (imu.new_nb2!=0)
			{
				filter2.dead_reckoning(imu.nb2_p,imu.nb2_q,imu.nb2_r,encoders.delta_distance,imu.dt2);
			}
			else
			{
				filter2.blind_dead_reckoning(imu.nb2_p,imu.nb2_q,imu.nb2_r,encoders.delta_distance,imu.dt2);
			}

			if (imu.new_nbS!=0)
			{
				filterS.dead_reckoning(imu.nb2_p,imu.nb2_q,imu.nb2_r,encoders.delta_distance,imu.dt2);
			}
			else
			{
				filterS.blind_dead_reckoning(imu.nb2_p,imu.nb2_q,imu.nb2_r,encoders.delta_distance,imu.dt2);
			}

			
			if(filter.keep_nb == 3 || filter.keep_nb == 28 || filter.keep_nb == 1 || filter.keep_nb == 5 || filter.keep_nb == 7 || filter.keep_nb == 9 || filter.keep_nb == 11 || filter.keep_nb == 12 || filter.keep_nb == 14 || filter.keep_nb == 20 || filter.keep_nb == 22 || filter.keep_nb == 24 || filter.keep_nb == 26 || filter.keep_nb == 29 || filter.keep_nb == 31 || filter.keep_nb == 33 || filter.keep_nb == 35)
			{
				filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			else if (filter.keep_nb == 2 || filter.keep_nb == 6 || filter.keep_nb == 8 || filter.keep_nb == 10 || filter.keep_nb == 15 || filter.keep_nb == 16 || filter.keep_nb == 18 || filter.keep_nb == 21 || filter.keep_nb == 23 || filter.keep_nb == 25 || filter.keep_nb == 27 || filter.keep_nb == 30 || filter.keep_nb == 32 || filter.keep_nb == 34 || filter.keep_nb == 36)
			{

				filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			else if (filter.keep_nb == 4 || filter.keep_nb == 13 || filter.keep_nb == 17 || filter.keep_nb == 19)
			{
				filter.initialize_states(filterS.phi,filterS.theta,filterS.psi,filterS.x,filterS.y,filterS.P_phi,filterS.P_theta,filterS.P_psi,filterS.P_x,filterS.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}
	}

	if (stopFlag && homing_found)
	{
		if(do_homing)
		{
			filter.verify_homing(homing_heading, homing_x, homing_y, possibly_lost);
			if(filter.heading_verified)
			{
				filter.initialize_states(filter.phi,filter.theta,filter.heading_update,homing_x,homing_y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
				filter1.initialize_states(filter.phi,filter.theta,filter.heading_update,homing_x,homing_y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
				filter2.initialize_states(filter.phi,filter.theta,filter.heading_update,homing_x,homing_y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
				filterS.initialize_states(filter.phi,filter.theta,filter.	heading_update,homing_x,homing_y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
				homing_updated = true;
			}
			filter.heading_verified = false;
		}
	}
	else if (!stopFlag)
	{
		homing_updated = false;
	}


    if ((imu.nb1_missed_counter>50 && imu.nb1_good || imu.nb2_missed_counter>50 && imu.nb2_good) && stop_request == false)
	{
		stop_request = true;
		stop_time = ros::Time::now().toSec();
	}
    else if (stop_request == true && ((imu.nb1_current && imu.nb2_current)||(imu.nb1_current && !imu.nb2_good)||(!imu.nb1_good && imu.nb2_current)))
	{
		stop_request = false;

		if(imu.nb1_current)
		{
			imu.nb1_good_prev = true;
		}
		if(imu.nb2_current)
		{
			imu.nb2_good_prev = true;
		}
		if(imu.nbS_current)
		{
			imu.nbS_good_prev = true;
		}

		if(filter.keep_nb == 1 || filter.keep_nb == 5 || filter.keep_nb == 7 || filter.keep_nb == 9 || filter.keep_nb == 11 || filter.keep_nb == 12 || filter.keep_nb == 14 || filter.keep_nb == 20 || filter.keep_nb == 22 || filter.keep_nb == 24 || filter.keep_nb == 26 || filter.keep_nb == 29 || filter.keep_nb == 31 || filter.keep_nb == 33 || filter.keep_nb == 35)
		{
			filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			if(filter.keep_nb == 1 || filter.keep_nb == 5 || filter.keep_nb == 7 || filter.keep_nb == 9)
			{
				filter2.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			if(filter.keep_nb == 1 || filter.keep_nb == 5 || filter.keep_nb == 7 || filter.keep_nb == 9 || filter.keep_nb == 11 || filter.keep_nb == 14)
			{
				filterS.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}
		else if (filter.keep_nb == 2 || filter.keep_nb == 6 || filter.keep_nb == 8 || filter.keep_nb == 10 || filter.keep_nb == 15 || filter.keep_nb == 16 || filter.keep_nb == 18 || filter.keep_nb == 21 || filter.keep_nb == 23 || filter.keep_nb == 25 || filter.keep_nb == 27 || filter.keep_nb == 30 || filter.keep_nb == 32 || filter.keep_nb == 34 || filter.keep_nb == 36)
		{

			filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			if(filter.keep_nb == 2 || filter.keep_nb == 6 || filter.keep_nb == 8 || filter.keep_nb == 10)
			{
				filter1.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			if(filter.keep_nb == 2 || filter.keep_nb == 6 || filter.keep_nb == 8 || filter.keep_nb == 10 || filter.keep_nb == 15 || filter.keep_nb == 18)
			{
				filterS.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}
		else if (filter.keep_nb == 3 || filter.keep_nb == 28)
		{
			filter.initialize_states((filter1.phi+filter2.phi)/2.0,(filter1.theta+filter2.theta)/2.0,(filter1.psi+filter2.psi)/2.0,(filter1.x+filter2.x)/2.0,(filter1.y+filter2.y)/2.0,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			filter1.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			filter2.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			if (filter.keep_nb == 3)
			{
				filterS.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}
		else if (filter.keep_nb == 4 || filter.keep_nb == 13 || filter.keep_nb == 17 || filter.keep_nb == 19)
		{
			filter.initialize_states(filterS.phi,filterS.theta,filterS.psi,filterS.x,filterS.y,filterS.P_phi,filterS.P_theta,filterS.P_psi,filterS.P_x,filterS.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			if (filter.keep_nb == 4 || filter.keep_nb == 13)
			{
				filter1.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
			if (filter.keep_nb == 4 || filter.keep_nb == 17)
			{
				filter2.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
			}
		}
	}
    else if (ros::Time::now().toSec()-stop_time>20.0)
	{
		stop_request = false;
        if (imu.nb1_missed_counter>50)
		{
			 imu.nb1_good = false;
			 imu.nb1_good_prev = false;
		}
        if (imu.nb2_missed_counter>50)
		{
			 imu.nb2_good = false;
			 imu.nb2_good_prev = false;
		}
		if (imu.nbS_missed_counter>50)
		{
			 imu.nbS_good = false;
			 imu.nbS_good_prev = false;
		}
	}

	if (latest_nav_control_request.runBiasRemoval && collected_gyro_data)
	{
		nav_status_output = 1;
	}
	else
	{
		nav_status_output = 0;
	}

}

void NavigationFilter::getExecInfoCallback(const messages::ExecInfo::ConstPtr &msg)
{
	this->pause_switch = msg->pause;
	this->turnFlag = msg->turnFlag;
	this->stopFlag = msg->stopFlag;
}

void NavigationFilter::getMissionPlanningInfoCallback(const messages::MissionPlanningInfo::ConstPtr &msg)
{
	this->possibly_lost = msg->possiblyLost;
}

void NavigationFilter::getLidarFilterOutCallback(const messages::LidarFilterOut::ConstPtr &msg)
{
	double l_dist = 0.44;
	this->homing_x = msg->homing_x-l_dist*cos(msg->homing_heading);
	this->homing_y = msg->homing_y-l_dist*sin(msg->homing_heading);
	this->homing_heading = msg->homing_heading;
	this->homing_found = msg->homing_found;
}

//added for new User Interface -Matt G.
bool NavigationFilter::navFilterControlServiceCallback(messages::NavFilterControl::Request &request, messages::NavFilterControl::Response &response)
{
    this->latest_nav_control_request = request;
    response.p1Offset = imu.p1_offset;
    response.q1Offset = imu.q1_offset;
    response.r1Offset = imu.r1_offset;

    response.p2Offset = imu.p2_offset;
    response.q2Offset = imu.q2_offset;
    response.r2Offset = imu.r2_offset;

    response.p3Offset = imu.p3_offset;
    response.q3Offset = imu.q3_offset;
    response.r3Offset = imu.r3_offset;
    
    if(request.setInitNorthAngle) init_north_angle = request.initNorthAngle*DEG_2_RAD;

    if(request.setGlobalPose)
    {
        ROS_INFO("Teleporting to new Location");
        filter.initialize_states(filter.phi,filter.theta,request.newHeading*DEG_2_RAD,request.newX,request.newY,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
		filter1.initialize_states(filter.phi,filter.theta,request.newHeading*DEG_2_RAD,request.newX,request.newY,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
    	filter2.initialize_states(filter.phi,filter.theta,request.newHeading*DEG_2_RAD,request.newX,request.newY,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
    	filterS.initialize_states(filter.phi,filter.theta,request.newHeading*DEG_2_RAD,request.newX,request.newY,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
    	
    	homing_updated = true; // to update the SLAM
    }
    if(request.setNorthAngle)
    {
        ROS_INFO("Setting New North Angle");
        filter.north_angle = request.northAngle * DEG_2_RAD;
        filter1.north_angle = request.northAngle * DEG_2_RAD;
        filter2.north_angle = request.northAngle * DEG_2_RAD;
        filterS.north_angle = request.northAngle * DEG_2_RAD;

    }
    
    return true;
}