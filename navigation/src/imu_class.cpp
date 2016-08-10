#include <navigation/imu_class.hpp>


IMU::IMU()
{
	start_time = ros::Time::now().toSec();

	//combined imus
	p=0;
	q=0;
	r=0;
	ax=0;
	ay=0;
	az=0;
	new_imu = 0;
	bias_thresh = 0.01;

	//nb1
	p1=0;
	q1=0;
	r1=0;
	ax1=0;
	ay1=0;
	az1=0;
	p1_offset = 0.000151251102727;
	q1_offset = -0.001028705454545;
	r1_offset = 0.00049114595454;
	E_p1_offset = 0.000151251102727;
	E_q1_offset = -0.001028705454545;
	E_r1_offset = 0.00049114595454;
	mean_p1 = 0;
	mean_q1 = 0;
	mean_r1 = 0;
	good_bias1 = true;
	p1_values.clear();
	q1_values.clear();
	r1_values.clear();
	num_imus1=0;
	nb1_counter=0;
	nb1_counter_prev = 0;
	call_counter1=0;
	prev_time1 = 0;
	dt1 = 0;
	new_imu1 = 0;
	subscriber_imu1 = node.subscribe("hw_interface/nb1in/nb1in", 1, &IMU::getIMU1Callback,this);

	//nb2
	p2=0;
	q2=0;
	r2=0;
	ax2=0;
	ay2=0;
	az2=0;
	p2_offset = 0.000548155500000;
	q2_offset = -0.001647460000000;
	r2_offset = 0.002133171818182;
	E_p2_offset = 0.000548155500000;
	E_q2_offset = -0.001647460000000;
	E_r2_offset = 0.002133171818182;
	mean_p2 = 0;
	mean_q2 = 0;
	mean_r2 = 0;
	good_bias2 = true;
	p2_values.clear();
	q2_values.clear();
	r2_values.clear();
	num_imus2=0;
	nb2_counter=0;
	nb2_counter_prev = 0;
	call_counter2=0;
	time2=0;
	prev_time2 = 0;
	dt2 = 0;
	new_imu2 = 0;
	subscriber_imu2 = node.subscribe("hw_interface/nb2in/nb2in", 1, &IMU::getIMU2Callback,this);

	//nb3
	p3=0;
	q3=0;
	r3=0;
	ax3=0;
	ay3=0;
	az3=0;
	p3_offset = 0.002108505909091;
	q3_offset = 0.001887883636364;
	r3_offset = 0.003590465000000;
	E_p3_offset = 0.002108505909091;
	E_q3_offset = 0.001887883636364;
	E_r3_offset = 0.003590465000000;
	mean_p3 = 0;
	mean_q3 = 0;
	mean_r3 = 0;
	good_bias3 = true;
	p3_values.clear();
	q3_values.clear();
	r3_values.clear();
	num_imus3=0;
	nb3_counter=0;
	nb3_counter_prev = 0;
	call_counter3=0;
	time3=0;
	prev_time3 = 0;
	dt3 = 0;
	new_imu3 = 0;
	subscriber_imu3 = node.subscribe("hw_interface/nb3in/nb3in", 1, &IMU::getIMU3Callback,this);
}

void IMU::collect_gyro1_data()
{
	if (new_imu1!=0)
	{
		p1_values = arma::join_vert(p1_values,arma::mat(1,1,arma::fill::ones)*(p1+p1_offset));
		q1_values = arma::join_vert(q1_values,arma::mat(1,1,arma::fill::ones)*(q1+q1_offset));
		r1_values = arma::join_vert(r1_values,arma::mat(1,1,arma::fill::ones)*(r1+r1_offset));
	}
}

void IMU::collect_gyro2_data()
{
	if (new_imu2!=0)
	{
		p2_values = arma::join_vert(p2_values,arma::mat(1,1,arma::fill::ones)*(p2+p2_offset));
		q2_values = arma::join_vert(q2_values,arma::mat(1,1,arma::fill::ones)*(q2+q2_offset));
		r2_values = arma::join_vert(r2_values,arma::mat(1,1,arma::fill::ones)*(r2+r2_offset));
	}
}

void IMU::collect_gyro3_data()
{
	if (new_imu3!=0)
	{
		p3_values = arma::join_vert(p3_values,arma::mat(1,1,arma::fill::ones)*(p3+p3_offset));
		q3_values = arma::join_vert(q3_values,arma::mat(1,1,arma::fill::ones)*(q3+q3_offset));
		r3_values = arma::join_vert(r3_values,arma::mat(1,1,arma::fill::ones)*(r3+r3_offset));
	}
}

void IMU::calculate_gyro1_offset()
{
	arma::mat p_holder1 = p1_values;
	arma::mat q_holder1 = q1_values;
	arma::mat r_holder1 = r1_values;
	arma::mat p_holder2;
	arma::mat q_holder2;
	arma::mat r_holder2;
	arma::mat p_meds;
	arma::mat q_meds;
	arma::mat r_meds;
	arma::mat m_temp;
	good_bias1 = true;
	
	// p_est
	double m_p = 0;
	int s_p = 0;
	double min_dist_p = 0;
	int min_index_p = 0;
	p_holder2.clear();
	p_meds.clear();
	for (int jj = 0; jj<25; jj++)
	{
		m_temp = arma::median(arma::median(p_holder1));
		m_p = m_temp(0,0);
		s_p = p_holder1.size();
		min_dist_p = fabs(p_holder1(0,0)-m_p);
		min_index_p = 0;
		for (int ii = 1; ii<s_p; ii++)
		{
			if (fabs(p_holder1(ii,0)-m_p) < min_dist_p)
			{
				min_dist_p = fabs(p_holder1(ii,0)-m_p);
				min_index_p = ii;
			}
		}
		
		for (int ii = 0; ii<s_p; ii++)
		{
			if (ii != min_index_p)
			{
				p_holder2 = arma::join_vert(p_holder2,arma::mat(1,1,arma::fill::ones)*p_holder1(ii,0));
			}
			else
			{
				p_meds = arma::join_vert(p_meds,arma::mat(1,1,arma::fill::ones)*p_holder1(ii,0));
			}
		}
		p_holder1.clear();
		p_holder1 = p_holder2;
		p_holder2.clear();
	}
	mean_p1 = arma::mean(arma::mean(p_meds));
	if (fabs(mean_p1-E_p1_offset)>bias_thresh)
	{
		good_bias1 = false;
	}

	// q_est
	double m_q = 0;
	int s_q = 0;
	double min_dist_q = 0;
	int min_index_q = 0;
	q_holder2.clear();
	q_meds.clear();
	for (int jj = 0; jj<25; jj++)
	{
		m_temp = arma::median(arma::median(q_holder1));
		m_q = m_temp(0,0);
		s_q = q_holder1.size();
		min_dist_q = fabs(q_holder1(0,0)-m_q);
		min_index_q = 0;
		for (int ii = 1; ii<s_q; ii++)
		{
			if (fabs(q_holder1(ii,0)-m_q) < min_dist_q)
			{
				min_dist_q = fabs(q_holder1(ii,0)-m_q);
				min_index_q = ii;
			}
		}
		
		for (int ii = 0; ii<s_q; ii++)
		{
			if (ii != min_index_q)
			{
				q_holder2 = arma::join_vert(q_holder2,arma::mat(1,1,arma::fill::ones)*q_holder1(ii,0));
			}
			else
			{
				q_meds = arma::join_vert(q_meds,arma::mat(1,1,arma::fill::ones)*q_holder1(ii,0));
			}
		}
		q_holder1.clear();
		q_holder1 = q_holder2;
		q_holder2.clear();
	}
	mean_q1 = arma::mean(arma::mean(q_meds));
	if (fabs(mean_q1-E_q1_offset)>bias_thresh)
	{
		good_bias1 = false;
	}

	// r_est
	double m_r = 0;
	int s_r = 0;
	double min_dist_r = 0;
	int min_index_r = 0;
	r_holder2.clear();
	r_meds.clear();
	for (int jj = 0; jj<25; jj++)
	{
		m_temp = arma::median(arma::median(r_holder1));
		m_r = m_temp(0,0);
		s_r = r_holder1.size();
		min_dist_r = fabs(r_holder1(0,0)-m_r);
		min_index_r = 0;
		for (int ii = 1; ii<s_r; ii++)
		{
			if (fabs(r_holder1(ii,0)-m_r) < min_dist_r)
			{
				min_dist_r = fabs(r_holder1(ii,0)-m_r);
				min_index_r = ii;
			}
		}
		
		for (int ii = 0; ii<s_r; ii++)
		{
			if (ii != min_index_r)
			{
				r_holder2 = arma::join_vert(r_holder2,arma::mat(1,1,arma::fill::ones)*r_holder1(ii,0));
			}
			else
			{
				r_meds = arma::join_vert(r_meds,arma::mat(1,1,arma::fill::ones)*r_holder1(ii,0));
			}
		}
		r_holder1.clear();
		r_holder1 = r_holder2;
		r_holder2.clear();
	}
	mean_r1 = arma::mean(arma::mean(r_meds));
	if (fabs(mean_r1-E_r1_offset)>bias_thresh)
	{
		good_bias1 = false;
	}
	
	/*m_temp = arma::median(arma::median(p1_values));
	p1_offset = m_temp(0,0);
	m_temp = arma::median(arma::median(q1_values));
	q1_offset = m_temp(0,0);
	m_temp = arma::median(arma::median(r1_values));
	r1_offset = m_temp(0,0);*/
}

void IMU::calculate_gyro2_offset()
{
	arma::mat p_holder1 = p2_values;
	arma::mat q_holder1 = q2_values;
	arma::mat r_holder1 = r2_values;
	arma::mat p_holder2;
	arma::mat q_holder2;
	arma::mat r_holder2;
	arma::mat p_meds;
	arma::mat q_meds;
	arma::mat r_meds;
	arma::mat m_temp;
	good_bias2 = true;
	
	// p_est
	double m_p = 0;
	int s_p = 0;
	double min_dist_p = 0;
	int min_index_p = 0;
	p_holder2.clear();
	p_meds.clear();
	for (int jj = 0; jj<25; jj++)
	{
		m_temp = arma::median(arma::median(p_holder1));
		m_p = m_temp(0,0);
		s_p = p_holder1.size();
		min_dist_p = fabs(p_holder1(0,0)-m_p);
		min_index_p = 0;
		for (int ii = 1; ii<s_p; ii++)
		{
			if (fabs(p_holder1(ii,0)-m_p) < min_dist_p)
			{
				min_dist_p = fabs(p_holder1(ii,0)-m_p);
				min_index_p = ii;
			}
		}
		
		for (int ii = 0; ii<s_p; ii++)
		{
			if (ii != min_index_p)
			{
				p_holder2 = arma::join_vert(p_holder2,arma::mat(1,1,arma::fill::ones)*p_holder1(ii,0));
			}
			else
			{
				p_meds = arma::join_vert(p_meds,arma::mat(1,1,arma::fill::ones)*p_holder1(ii,0));
			}
		}
		p_holder1.clear();
		p_holder1 = p_holder2;
		p_holder2.clear();
	}
	mean_p2 = arma::mean(arma::mean(p_meds));
	if (fabs(mean_p2-E_p2_offset)>bias_thresh)
	{
		good_bias2 = false;
	}

	// q_est
	double m_q = 0;
	int s_q = 0;
	double min_dist_q = 0;
	int min_index_q = 0;
	q_holder2.clear();
	q_meds.clear();
	for (int jj = 0; jj<25; jj++)
	{
		m_temp = arma::median(arma::median(q_holder1));
		m_q = m_temp(0,0);
		s_q = q_holder1.size();
		min_dist_q = fabs(q_holder1(0,0)-m_q);
		min_index_q = 0;
		for (int ii = 1; ii<s_q; ii++)
		{
			if (fabs(q_holder1(ii,0)-m_q) < min_dist_q)
			{
				min_dist_q = fabs(q_holder1(ii,0)-m_q);
				min_index_q = ii;
			}
		}
		
		for (int ii = 0; ii<s_q; ii++)
		{
			if (ii != min_index_q)
			{
				q_holder2 = arma::join_vert(q_holder2,arma::mat(1,1,arma::fill::ones)*q_holder1(ii,0));
			}
			else
			{
				q_meds = arma::join_vert(q_meds,arma::mat(1,1,arma::fill::ones)*q_holder1(ii,0));
			}
		}
		q_holder1.clear();
		q_holder1 = q_holder2;
		q_holder2.clear();
	}
	mean_q2 = arma::mean(arma::mean(q_meds));
	if (fabs(mean_q2-E_q2_offset)>bias_thresh)
	{
		good_bias2 = false;
	}

	// r_est
	double m_r = 0;
	int s_r = 0;
	double min_dist_r = 0;
	int min_index_r = 0;
	r_holder2.clear();
	r_meds.clear();
	for (int jj = 0; jj<25; jj++)
	{
		m_temp = arma::median(arma::median(r_holder1));
		m_r = m_temp(0,0);
		s_r = r_holder1.size();
		min_dist_r = fabs(r_holder1(0,0)-m_r);
		min_index_r = 0;
		for (int ii = 1; ii<s_r; ii++)
		{
			if (fabs(r_holder1(ii,0)-m_r) < min_dist_r)
			{
				min_dist_r = fabs(r_holder1(ii,0)-m_r);
				min_index_r = ii;
			}
		}
		
		for (int ii = 0; ii<s_r; ii++)
		{
			if (ii != min_index_r)
			{
				r_holder2 = arma::join_vert(r_holder2,arma::mat(1,1,arma::fill::ones)*r_holder1(ii,0));
			}
			else
			{
				r_meds = arma::join_vert(r_meds,arma::mat(1,1,arma::fill::ones)*r_holder1(ii,0));
			}
		}
		r_holder1.clear();
		r_holder1 = r_holder2;
		r_holder2.clear();
	}
	mean_r2 = arma::mean(arma::mean(r_meds));
	if (fabs(mean_r2-E_r2_offset)>bias_thresh)
	{
		good_bias2 = false;
	}
	
	/*m_temp = arma::median(arma::median(p2_values));
	p2_offset = m_temp(0,0);
	m_temp = arma::median(arma::median(q2_values));
	q2_offset = m_temp(0,0);
	m_temp = arma::median(arma::median(r2_values));
	r2_offset = m_temp(0,0);*/
}

void IMU::calculate_gyro3_offset()
{
	arma::mat p_holder1 = p3_values;
	arma::mat q_holder1 = q3_values;
	arma::mat r_holder1 = r3_values;
	arma::mat p_holder2;
	arma::mat q_holder2;
	arma::mat r_holder2;
	arma::mat p_meds;
	arma::mat q_meds;
	arma::mat r_meds;
	arma::mat m_temp;
	good_bias3 = true;
	
	// p_est
	double m_p = 0;
	int s_p = 0;
	double min_dist_p = 0;
	int min_index_p = 0;
	p_holder2.clear();
	p_meds.clear();
	for (int jj = 0; jj<25; jj++)
	{
		m_temp = arma::median(arma::median(p_holder1));
		m_p = m_temp(0,0);
		s_p = p_holder1.size();
		min_dist_p = fabs(p_holder1(0,0)-m_p);
		min_index_p = 0;
		for (int ii = 1; ii<s_p; ii++)
		{
			if (fabs(p_holder1(ii,0)-m_p) < min_dist_p)
			{
				min_dist_p = fabs(p_holder1(ii,0)-m_p);
				min_index_p = ii;
			}
		}
		
		for (int ii = 0; ii<s_p; ii++)
		{
			if (ii != min_index_p)
			{
				p_holder2 = arma::join_vert(p_holder2,arma::mat(1,1,arma::fill::ones)*p_holder1(ii,0));
			}
			else
			{
				p_meds = arma::join_vert(p_meds,arma::mat(1,1,arma::fill::ones)*p_holder1(ii,0));
			}
		}
		p_holder1.clear();
		p_holder1 = p_holder2;
		p_holder2.clear();
	}
	mean_p3 = arma::mean(arma::mean(p_meds));
	if (fabs(mean_p3-E_p3_offset)>bias_thresh)
	{
		good_bias3 = false;
	}

	// q_est
	double m_q = 0;
	int s_q = 0;
	double min_dist_q = 0;
	int min_index_q = 0;
	q_holder2.clear();
	q_meds.clear();
	for (int jj = 0; jj<25; jj++)
	{
		m_temp = arma::median(arma::median(q_holder1));
		m_q = m_temp(0,0);
		s_q = q_holder1.size();
		min_dist_q = fabs(q_holder1(0,0)-m_q);
		min_index_q = 0;
		for (int ii = 1; ii<s_q; ii++)
		{
			if (fabs(q_holder1(ii,0)-m_q) < min_dist_q)
			{
				min_dist_q = fabs(q_holder1(ii,0)-m_q);
				min_index_q = ii;
			}
		}
		
		for (int ii = 0; ii<s_q; ii++)
		{
			if (ii != min_index_q)
			{
				q_holder2 = arma::join_vert(q_holder2,arma::mat(1,1,arma::fill::ones)*q_holder1(ii,0));
			}
			else
			{
				q_meds = arma::join_vert(q_meds,arma::mat(1,1,arma::fill::ones)*q_holder1(ii,0));
			}
		}
		q_holder1.clear();
		q_holder1 = q_holder2;
		q_holder2.clear();
	}
	mean_q3 = arma::mean(arma::mean(q_meds));
	if (fabs(mean_q3-E_q3_offset)>bias_thresh)
	{
		good_bias3 = false;
	}

	// r_est
	double m_r = 0;
	int s_r = 0;
	double min_dist_r = 0;
	int min_index_r = 0;
	r_holder2.clear();
	r_meds.clear();
	for (int jj = 0; jj<25; jj++)
	{
		m_temp = arma::median(arma::median(r_holder1));
		m_r = m_temp(0,0);
		s_r = r_holder1.size();
		min_dist_r = fabs(r_holder1(0,0)-m_r);
		min_index_r = 0;
		for (int ii = 1; ii<s_r; ii++)
		{
			if (fabs(r_holder1(ii,0)-m_r) < min_dist_r)
			{
				min_dist_r = fabs(r_holder1(ii,0)-m_r);
				min_index_r = ii;
			}
		}
		
		for (int ii = 0; ii<s_r; ii++)
		{
			if (ii != min_index_r)
			{
				r_holder2 = arma::join_vert(r_holder2,arma::mat(1,1,arma::fill::ones)*r_holder1(ii,0));
			}
			else
			{
				r_meds = arma::join_vert(r_meds,arma::mat(1,1,arma::fill::ones)*r_holder1(ii,0));
			}
		}
		r_holder1.clear();
		r_holder1 = r_holder2;
		r_holder2.clear();
	}
	mean_r3 = arma::mean(arma::mean(r_meds));
	if (fabs(mean_r3-E_r3_offset)>bias_thresh)
	{
		good_bias3 = false;
	}
	
	/*m_temp = arma::median(arma::median(p3_values));
	p3_offset = m_temp(0,0);
	m_temp = arma::median(arma::median(q3_values));
	q3_offset = m_temp(0,0);
	m_temp = arma::median(arma::median(r3_values));
	r3_offset = m_temp(0,0);*/
}

void IMU::clear_gyro1_values()
{
	p1_values.clear();
	q1_values.clear();
	r1_values.clear();
}

void IMU::clear_gyro2_values()
{
	p2_values.clear();
	q2_values.clear();
	r2_values.clear();
}

void IMU::clear_gyro3_values()
{
	p3_values.clear();
	q3_values.clear();
	r3_values.clear();
}

void IMU::filter_imu_values()
{
		p1 = p1-p1_offset;
		q1 = q1-q1_offset;
		r1 = r1-r1_offset;
		if (prev_time1!=0)
		{
			dt1 = time1 - prev_time1;
		}
		else 
		{
			dt1 = 0.0;
		}
		if (fabs(dt1)>1000.0)
		{
			dt1 = 0.0;
		}


		p2 = p2-p2_offset;
		q2 = q2-q2_offset;
		r2 = r2-r2_offset;
		if (prev_time2!=0)
		{
			dt2 = time2 - prev_time2;
		}
		else 
		{
			dt2 = 0.0;
		}
		if (fabs(dt2)>1000.0)
		{
			dt2 = 0.0;
		}

		p3 = p3-p3_offset;
		q3 = q3-q3_offset;
		r3 = r3-r3_offset;
		if (prev_time3!=0)
		{
			dt3 = time3 - prev_time3;
		}
		else 
		{
			dt3 = 0.0;
		}
		if (fabs(dt3)>1000.0)
		{
			dt3 = 0.0;
		}


	if (new_imu == 1)
	{
		p = ((double)new_imu1*(p1)+(double)new_imu2*(p2)+(double)new_imu3*(p3))/((double)new_imu1+(double)new_imu2+(double)new_imu3);
		q = ((double)new_imu1*(q1)+(double)new_imu2*(q2)+(double)new_imu3*(q3))/((double)new_imu1+(double)new_imu2+(double)new_imu3);
		r = ((double)new_imu1*(r1)+(double)new_imu2*(r2)+(double)new_imu3*(r3))/((double)new_imu1+(double)new_imu2+(double)new_imu3);
		ax = ((double)new_imu1*(ax1)+(double)new_imu2*(ax2)+(double)new_imu3*(ax3))/((double)new_imu1+(double)new_imu2+(double)new_imu3);
		ay = ((double)new_imu1*(ay1)+(double)new_imu2*(ay2)+(double)new_imu3*(ay3))/((double)new_imu1+(double)new_imu2+(double)new_imu3);
		az = ((double)new_imu1*(az1)+(double)new_imu2*(az2)+(double)new_imu3*(az3))/((double)new_imu1+(double)new_imu2+(double)new_imu3);

	}
}

void IMU::set_prev_counters()
{
	nb1_counter_prev = nb1_counter;
	nb2_counter_prev = nb2_counter;
	nb3_counter_prev = nb3_counter;
	prev_time1 = time1;
	prev_time2 = time2;
	prev_time3 = time3;
}

void IMU::set_gyro1_offset()
{
	p1_offset = mean_p1;
	q1_offset = mean_q1;
	r1_offset = mean_r1;
}

void IMU::set_gyro2_offset()
{
	p2_offset = mean_p2;
	q2_offset = mean_q2;
	r2_offset = mean_r2;
}

void IMU::set_gyro3_offset()
{
	p3_offset = mean_p3;
	q3_offset = mean_q3;
	r3_offset = mean_r3;
}

void IMU::determine_new_data()
{
	if (nb1_counter!=nb1_counter_prev && num_imus1 == 2)
	{
		new_imu1 = 1;
	}
	else
	{
		new_imu1 = 0;
	}

	if (nb2_counter!=nb2_counter_prev && num_imus2 == 2)
	{
		new_imu2 = 1;
	}
	else
	{
		new_imu2 = 0;
	}

	if (nb3_counter!=nb3_counter_prev && num_imus3 == 2)
	{
		new_imu3 = 1;
	}
	else
	{
		new_imu3 = 0;
	}

	if (new_imu1 != 0 || new_imu2 != 0 || new_imu3 != 0)
	{
		new_imu = 1;
	}
	else
	{
		new_imu = 0;
	}
}