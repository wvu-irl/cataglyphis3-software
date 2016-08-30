#include <navigation/imu_class.hpp>


IMU::IMU()
{
	start_time = ros::Time::now().toSec();

	//combined imus
	bias_thresh = 0.01;
	p=0;
	q=0;
	r=0;
	ax=0;
	ay=0;
	az=0;

	//netburner1
	nb1_p=0;
	nb1_q=0;
	nb1_r=0;
	nb1_ax=0;
	nb1_ay=0;
	nb1_az=0;
	nb1_imu_nums=0;
	nb1_num_imus=0;
	nb1_counter=0;
	nb1_counter_prev = 0;
	nb1_diff_prev = 1;
	nb1_drive_counter = 0;
    nb1_current = true;
	nb1_good = true;
	nb1_good_prev = true;
	call_counter1=0;
	prev_time1 = 0;
	dt1 = 0;
	new_nb1 = 0;
	subscriber_imu1 = node.subscribe("hw_interface/nb1in/nb1in", 1, &IMU::getIMU1Callback,this);

	//netburner2
	nb2_p=0;
	nb2_q=0;
	nb2_r=0;
	nb2_ax=0;
	nb2_ay=0;
	nb2_az=0;
	nb2_imu_nums=0;
	nb2_num_imus=0;
	nb2_counter=0;
	nb2_counter_prev = 0;
	nb2_diff_prev = 1;
	nb2_drive_counter = 0;
    nb2_current = true;
	nb2_good = true;
	nb2_good_prev = true;
	call_counter2=0;
	prev_time2 = 0;
	dt2 = 0;
	new_nb2 = 0;
	subscriber_imu2 = node.subscribe("hw_interface/nb2in/nb2in", 1, &IMU::getIMU2Callback,this);

	//netburner Serial
	nbS_p=0;
	nbS_q=0;
	nbS_r=0;
	nbS_ax=0;
	nbS_ay=0;
	nbS_az=0;
	nbS_counter=0;
	nbS_counter_prev = 0;
	nbS_diff_prev = 1;
	nbS_drive_counter = 0;
    nbS_current = true;
	nbS_good = true;
	nbS_good_prev = true;
	call_counterS=0;
	prev_timeS = 0;
	dtS = 0;
	new_nbS = 0;
	// subscriber_imuS = node.subscribe("hw_interface/nb2in/nb2in", 1, &IMU::getIMU2Callback,this);

	//imu1
	p1=0;
	q1=0;
	r1=0;
	ax1=0;
	ay1=0;
	az1=0;
	p1_offset = 0.00155578029808;
	q1_offset = 0.000418158248067;
	r1_offset = 0.000437778187916;
	E_p1_offset = 0.00155578029808;
	E_q1_offset = 0.000418158248067;
	E_r1_offset = 0.000437778187916;
	mean_p1 = 0;
	mean_q1 = 0;
	mean_r1 = 0;
	good_bias1 = true;
	p1_values.clear();
	q1_values.clear();
	r1_values.clear();
	new_imu1 = 0;
	imu_1_good = false;



	//imu2
	p2=0;
	q2=0;
	r2=0;
	ax2=0;
	ay2=0;
	az2=0;
	p2_offset = -0.000307843234623;
    q2_offset = 0.000208376179216;
	r2_offset = -0.00213066977449;
	E_p2_offset = -0.000307843234623;
    E_q2_offset = 0.000208376179216;
	E_r2_offset = -0.00213066977449;
	mean_p2 = 0;
	mean_q2 = 0;
	mean_r2 = 0;
	good_bias2 = true;
	p2_values.clear();
	q2_values.clear();
	r2_values.clear();
	new_imu2 = 0;
	imu_2_good = false;

	//imu3
	p3=0;
	q3=0;
	r3=0;
	ax3=0;
	ay3=0;
	az3=0;
	p3_offset = 0.000705252867192;
	q3_offset = -0.000602908607107;
	r3_offset = 0.00156109826639;
	E_p3_offset = 0.000705252867192;
	E_q3_offset = -0.000602908607107;
	E_r3_offset = 0.00156109826639;
	mean_p3 = 0;
	mean_q3 = 0;
	mean_r3 = 0;
	good_bias3 = true;
	p3_values.clear();
	q3_values.clear();
	r3_values.clear();
	new_imu3 = 0;
	imu_3_good = false;

	//imu4
	p4=0;
	q4=0;
	r4=0;
	ax4=0;
	ay4=0;
	az4=0;
	p4_offset = 0.000195449843886;
	q4_offset = -0.000753260857891;
	r4_offset = -0.00042131691589;
	E_p4_offset = 0.000195449843886;
	E_q4_offset = -0.000753260857891;
	E_r4_offset = -0.00042131691589;
	mean_p4 = 0;
	mean_q4 = 0;
	mean_r4 = 0;
	good_bias4 = true;
	p4_values.clear();
	q4_values.clear();
	r4_values.clear();
	new_imu4 = 0;
	imu_4_good = false;

	//imu5
	p5=0;
	q5=0;
	r5=0;
	ax5=0;
	ay5=0;
	az5=0;
	p5_offset = -0.00571636529639;
	q5_offset = 0.000290035066428;
	r5_offset = -0.00263201794587;
	E_p5_offset = -0.00571636529639;
	E_q5_offset = 0.000290035066428;
	E_r5_offset = -0.00263201794587;
	mean_p5 = 0;
	mean_q5 = 0;
	mean_r5 = 0;
	good_bias5 = true;
	p5_values.clear();
	q5_values.clear();
	r5_values.clear();
	new_imu5 = 0;
	imu_5_good = false;

	//imu6
	p6=0;
	q6=0;
	r6=0;
	ax6=0;
	ay6=0;
	az6=0;
	p6_offset = 0.000121627504996;
	q6_offset = 0.000229120953009;
	r6_offset = 0.000608300091699;
	E_p6_offset = 0.000121627504996;
	E_q6_offset = 0.000229120953009;
	E_r6_offset = 0.000608300091699;
	mean_p6 = 0;
	mean_q6 = 0;
	mean_r6 = 0;
	good_bias6 = true;
	p6_values.clear();
	q6_values.clear();
	r6_values.clear();
	new_imu6 = 0;
	imu_6_good = false;

	pS=0;
	qS=0;
	rS=0;
	axS=0;
	ayS=0;
	azS=0;
	pS_offset = 0.000548155500000;
	qS_offset = -0.001647460000000;
	rS_offset = 0.002133171818182;
	E_pS_offset = 0.000548155500000;
	E_qS_offset = -0.001647460000000;
	E_rS_offset = 0.002133171818182;
	mean_pS = 0;
	mean_qS = 0;
	mean_rS = 0;
	good_biasS = true;
	pS_values.clear();
	qS_values.clear();
	rS_values.clear();
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

void IMU::collect_gyro4_data()
{
	if (new_imu4!=0)
	{
		p4_values = arma::join_vert(p4_values,arma::mat(1,1,arma::fill::ones)*(p4+p4_offset));
		q4_values = arma::join_vert(q4_values,arma::mat(1,1,arma::fill::ones)*(q4+q4_offset));
		r4_values = arma::join_vert(r4_values,arma::mat(1,1,arma::fill::ones)*(r4+r4_offset));
	}
}

void IMU::collect_gyro5_data()
{
	if (new_imu5!=0)
	{
		p5_values = arma::join_vert(p5_values,arma::mat(1,1,arma::fill::ones)*(p5+p5_offset));
		q5_values = arma::join_vert(q5_values,arma::mat(1,1,arma::fill::ones)*(q5+q5_offset));
		r5_values = arma::join_vert(r5_values,arma::mat(1,1,arma::fill::ones)*(r5+r5_offset));
	}
}

void IMU::collect_gyro6_data()
{
	if (new_imu6!=0)
	{
		p6_values = arma::join_vert(p6_values,arma::mat(1,1,arma::fill::ones)*(p6+p6_offset));
		q6_values = arma::join_vert(q6_values,arma::mat(1,1,arma::fill::ones)*(q6+q6_offset));
		r6_values = arma::join_vert(r6_values,arma::mat(1,1,arma::fill::ones)*(r6+r6_offset));
	}
}

void IMU::collect_gyroS_data()
{
	if (new_nbS!=0)
	{
		pS_values = arma::join_vert(pS_values,arma::mat(1,1,arma::fill::ones)*(pS+pS_offset));
		qS_values = arma::join_vert(qS_values,arma::mat(1,1,arma::fill::ones)*(qS+qS_offset));
		rS_values = arma::join_vert(rS_values,arma::mat(1,1,arma::fill::ones)*(rS+rS_offset));
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
}


void IMU::calculate_gyro4_offset()
{
	arma::mat p_holder1 = p4_values;
	arma::mat q_holder1 = q4_values;
	arma::mat r_holder1 = r4_values;
	arma::mat p_holder2;
	arma::mat q_holder2;
	arma::mat r_holder2;
	arma::mat p_meds;
	arma::mat q_meds;
	arma::mat r_meds;
	arma::mat m_temp;
	good_bias4 = true;
	
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
	mean_p4 = arma::mean(arma::mean(p_meds));
	if (fabs(mean_p4-E_p4_offset)>bias_thresh)
	{
		good_bias4 = false;
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
	mean_q4 = arma::mean(arma::mean(q_meds));
	if (fabs(mean_q4-E_q4_offset)>bias_thresh)
	{
		good_bias4 = false;
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
	mean_r4 = arma::mean(arma::mean(r_meds));
	if (fabs(mean_r4-E_r4_offset)>bias_thresh)
	{
		good_bias4 = false;
	}
}

void IMU::calculate_gyro5_offset()
{
	arma::mat p_holder1 = p5_values;
	arma::mat q_holder1 = q5_values;
	arma::mat r_holder1 = r5_values;
	arma::mat p_holder2;
	arma::mat q_holder2;
	arma::mat r_holder2;
	arma::mat p_meds;
	arma::mat q_meds;
	arma::mat r_meds;
	arma::mat m_temp;
	good_bias5 = true;
	
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
	mean_p5 = arma::mean(arma::mean(p_meds));
	if (fabs(mean_p5-E_p5_offset)>bias_thresh)
	{
		good_bias5 = false;
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
	mean_q5 = arma::mean(arma::mean(q_meds));
	if (fabs(mean_q5-E_q5_offset)>bias_thresh)
	{
		good_bias5 = false;
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
	mean_r5 = arma::mean(arma::mean(r_meds));
	if (fabs(mean_r5-E_r5_offset)>bias_thresh)
	{
		good_bias5 = false;
	}
}

void IMU::calculate_gyro6_offset()
{
	arma::mat p_holder1 = p6_values;
	arma::mat q_holder1 = q6_values;
	arma::mat r_holder1 = r6_values;
	arma::mat p_holder2;
	arma::mat q_holder2;
	arma::mat r_holder2;
	arma::mat p_meds;
	arma::mat q_meds;
	arma::mat r_meds;
	arma::mat m_temp;
	good_bias6 = true;
	
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
	mean_p6 = arma::mean(arma::mean(p_meds));
	if (fabs(mean_p6-E_p6_offset)>bias_thresh)
	{
		good_bias6 = false;
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
	mean_q6 = arma::mean(arma::mean(q_meds));
	if (fabs(mean_q6-E_q6_offset)>bias_thresh)
	{
		good_bias6 = false;
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
	mean_r6 = arma::mean(arma::mean(r_meds));
	if (fabs(mean_r6-E_r6_offset)>bias_thresh)
	{
		good_bias6 = false;
	}
}

void IMU::calculate_gyroS_offset()
{
	arma::mat p_holder1 = pS_values;
	arma::mat q_holder1 = qS_values;
	arma::mat r_holder1 = rS_values;
	arma::mat p_holder2;
	arma::mat q_holder2;
	arma::mat r_holder2;
	arma::mat p_meds;
	arma::mat q_meds;
	arma::mat r_meds;
	arma::mat m_temp;
	good_biasS = true;
	
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
	mean_pS = arma::mean(arma::mean(p_meds));
	if (fabs(mean_pS-E_pS_offset)>bias_thresh)
	{
		good_biasS = false;
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
	mean_qS = arma::mean(arma::mean(q_meds));
	if (fabs(mean_qS-E_qS_offset)>bias_thresh)
	{
		good_biasS = false;
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
	mean_rS = arma::mean(arma::mean(r_meds));
	if (fabs(mean_rS-E_rS_offset)>bias_thresh)
	{
		good_biasS = false;
	}
}

void IMU::clear_gyro_values()
{
	p1_values.clear();
	q1_values.clear();
	r1_values.clear();

	p2_values.clear();
	q2_values.clear();
	r2_values.clear();

	p3_values.clear();
	q3_values.clear();
	r3_values.clear();

	p4_values.clear();
	q4_values.clear();
	r4_values.clear();

	p5_values.clear();
	q5_values.clear();
	r5_values.clear();

	p6_values.clear();
	q6_values.clear();
	r6_values.clear();

	pS_values.clear();
	qS_values.clear();
	rS_values.clear();
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

void IMU::clear_gyro4_values()
{
	p4_values.clear();
	q4_values.clear();
	r4_values.clear();
}

void IMU::clear_gyro5_values()
{
	p5_values.clear();
	q5_values.clear();
	r5_values.clear();
}

void IMU::clear_gyro6_values()
{
	p6_values.clear();
	q6_values.clear();
	r6_values.clear();
}

void IMU::clear_gyroS_values()
{
	pS_values.clear();
	qS_values.clear();
	rS_values.clear();
}


void IMU::set_gyro1_offset()
{
	if(good_bias1)
	{
		p1_offset = mean_p1;
		q1_offset = mean_q1;
		r1_offset = mean_r1;
	}
}

void IMU::set_gyro2_offset()
{
	if(good_bias2)
	{
		p2_offset = mean_p2;
		q2_offset = mean_q2;
		r2_offset = mean_r2;
	}
}

void IMU::set_gyro3_offset()
{
	if(good_bias3)
	{
		p3_offset = mean_p3;
		q3_offset = mean_q3;
		r3_offset = mean_r3;
	}
}

void IMU::set_gyro4_offset()
{
	if(good_bias4)
	{
		p4_offset = mean_p4;
		q4_offset = mean_q4;
		r4_offset = mean_r4;
	}
}

void IMU::set_gyro5_offset()
{
    if(good_bias5)
	{
		p5_offset = mean_p5;
		q5_offset = mean_q5;
		r5_offset = mean_r5;
	}
}

void IMU::set_gyro6_offset()
{
	if(good_bias6)
	{
		p6_offset = mean_p6;
		q6_offset = mean_q6;
		r6_offset = mean_r6;
	}
}

void IMU::set_gyroS_offset()
{
	if(good_biasS)
	{
		pS_offset = mean_pS;
		qS_offset = mean_qS;
		rS_offset = mean_rS;
	}
}

void IMU::determine_new_data()
{
	if (nb1_counter!=nb1_counter_prev)
	{
		new_nb1 = 1;
	}
	else
	{
		new_nb1 = 0;
	}

	if (nb2_counter!=nb2_counter_prev)
	{
		new_nb2 = 1;
	}
	else
	{
		new_nb2 = 0;
	}

	if (nbS_counter!=nbS_counter_prev)
	{
		new_nbS = 1;
	}
	else
	{
		new_nbS = 0;
	}
}

void IMU::filter_imu_values()
{
		
	if (new_nb1 == 1)
	{
		if (prev_time1!=0)
		{
			dt1 = time1 - prev_time1;
		}
		else 
		{
			dt1 = 0.0;
		}
        if (fabs(dt1)>0.1)
		{
            dt1 = 0.1;
		}

		new_imu1 = (short int) imu_1_good;
		new_imu2 = (short int) imu_2_good;
		new_imu3 = (short int) imu_3_good;

		p1 = p1-p1_offset;
		q1 = q1-q1_offset;
		r1 = r1-r1_offset;

		p2 = p2-p2_offset;
		q2 = q2-q2_offset;
		r2 = r2-r2_offset;

		p3 = p3-p3_offset;
		q3 = q3-q3_offset;
		r3 = r3-r3_offset;

		nb1_num_imus = new_imu1+new_imu2+new_imu3;

		if (nb1_num_imus>0)
		{
			nb1_p = ((double)new_imu1*(p1)+(double)new_imu2*(p2)+(double)new_imu3*(p3))/((double)new_imu1+(double)new_imu2+(double)new_imu3);
			nb1_q = ((double)new_imu1*(q1)+(double)new_imu2*(q2)+(double)new_imu3*(q3))/((double)new_imu1+(double)new_imu2+(double)new_imu3);
			nb1_r = ((double)new_imu1*(r1)+(double)new_imu2*(r2)+(double)new_imu3*(r3))/((double)new_imu1+(double)new_imu2+(double)new_imu3);
			nb1_ax = ((double)new_imu1*(ax1)+(double)new_imu2*(ax2)+(double)new_imu3*(ax3))/((double)new_imu1+(double)new_imu2+(double)new_imu3);
			nb1_ay = ((double)new_imu1*(ay1)+(double)new_imu2*(ay2)+(double)new_imu3*(ay3))/((double)new_imu1+(double)new_imu2+(double)new_imu3);
			nb1_az = ((double)new_imu1*(az1)+(double)new_imu2*(az2)+(double)new_imu3*(az3))/((double)new_imu1+(double)new_imu2+(double)new_imu3);
		}
		else
		{
			nb1_p = 0;
			nb1_q = 0;
			nb1_r = 0;
			nb1_ax = 0;
			nb1_ay = 0;
			nb1_az = 0;
		}

	}
	else
	{
        dt1 = 0;
        nb1_p = 0;
		nb1_q = 0;
		nb1_r = 0;
		nb1_ax = 0;
		nb1_ay = 0;
		nb1_az = 0;
		new_imu1 = 0;
		new_imu2 = 0;
		new_imu3 = 0;
		nb1_num_imus = 0;
	}

	if (new_nb2 == 1)
	{
		if (prev_time2!=0)
		{
			dt2 = time2 - prev_time2;
		}
		else 
		{
			dt2 = 0.0;
		}
        if (fabs(dt2)>0.1)
		{
            dt2 = 0.1;
		}

		new_imu4 = (short int) imu_4_good;
		new_imu5 = (short int) imu_5_good;
		new_imu6 = (short int) imu_6_good;

		p4 = p4-p4_offset;
		q4 = q4-q4_offset;
		r4 = r4-r4_offset;

		p5 = p5-p5_offset;
		q5 = q5-q5_offset;
		r5 = r5-r5_offset;

		p6 = p6-p6_offset;
		q6 = q6-q6_offset;
		r6 = r6-r6_offset;

		nb2_num_imus = new_imu4+new_imu5+new_imu6;

		if (nb2_num_imus>0)
		{
			nb2_p = ((double)new_imu4*(p4)+(double)new_imu5*(p5)+(double)new_imu6*(p6))/((double)new_imu4+(double)new_imu5+(double)new_imu6);
			nb2_q = ((double)new_imu4*(q4)+(double)new_imu5*(q5)+(double)new_imu6*(q6))/((double)new_imu4+(double)new_imu5+(double)new_imu6);
			nb2_r = ((double)new_imu4*(r4)+(double)new_imu5*(r5)+(double)new_imu6*(r6))/((double)new_imu4+(double)new_imu5+(double)new_imu6);
			nb2_ax = ((double)new_imu4*(ax4)+(double)new_imu5*(ax5)+(double)new_imu6*(ax6))/((double)new_imu4+(double)new_imu5+(double)new_imu6);
			nb2_ay = ((double)new_imu4*(ay4)+(double)new_imu5*(ay5)+(double)new_imu6*(ay6))/((double)new_imu4+(double)new_imu5+(double)new_imu6);
			nb2_az = ((double)new_imu4*(az4)+(double)new_imu5*(az5)+(double)new_imu6*(az6))/((double)new_imu4+(double)new_imu5+(double)new_imu6);
		}
		else
		{
			nb2_p = 0;
			nb2_q = 0;
			nb2_r = 0;
			nb2_ax = 0;
			nb2_ay = 0;
			nb2_az = 0;
		}
	}
	else
	{
        dt2 = 0;
        nb2_p = 0;
		nb2_q = 0;
		nb2_r = 0;
		nb2_ax = 0;
		nb2_ay = 0;
		nb2_az = 0;
		new_imu4 = 0;
		new_imu5 = 0;
		new_imu6 = 0;
		nb2_num_imus = 0;
	}

	if (new_nbS == 1)
	{
		if (prev_timeS!=0)
		{
			dtS = timeS - prev_timeS;
		}
		else 
		{
			dtS = 0.0;
		}
        if (fabs(dtS)>0.1)
		{
            dtS = 0.1;
		}

		pS = pS-pS_offset;
		qS = qS-qS_offset;
		rS = rS-rS_offset;
	}
	else
	{
        dtS = 0;
        nbS_p = 0;
		nbS_q = 0;
		nbS_r = 0;
		nbS_ax = 0;
		nbS_ay = 0;
		nbS_az = 0;
	}

	if (!nb1_good && !nb2_good)
	{	
		p = nbS_p;
	    q = nbS_q;
	    r = nbS_r;
	    ax = nbS_ax;
	    ay = nbS_ay;
	    az = nbS_az;
	}
	else
	{
		if (nb1_num_imus+nb2_num_imus!=0)
		{
			p = ((double)nb1_num_imus*(nb1_p)+(double)nb2_num_imus*(nb2_p))/((double)nb1_num_imus+(double)nb2_num_imus);
		    q = ((double)nb1_num_imus*(nb1_q)+(double)nb2_num_imus*(nb2_q))/((double)nb1_num_imus+(double)nb2_num_imus);
		    r = ((double)nb1_num_imus*(nb1_r)+(double)nb2_num_imus*(nb2_r))/((double)nb1_num_imus+(double)nb2_num_imus);
		    ax = ((double)nb1_num_imus*(nb1_ax)+(double)nb2_num_imus*(nb2_ax))/((double)nb1_num_imus+(double)nb2_num_imus);
		    ay = ((double)nb1_num_imus*(nb1_ay)+(double)nb2_num_imus*(nb2_ay))/((double)nb1_num_imus+(double)nb2_num_imus);
		    az = ((double)nb1_num_imus*(nb1_az)+(double)nb2_num_imus*(nb2_az))/((double)nb1_num_imus+(double)nb2_num_imus);
		}
		else
		{
			p = 0;
		    q = 0;
		    r = 0;
		    ax = 0;
		    ay = 0;
		    az = 0;
		}
	}

}

void IMU::set_prev_counters()
{
	if (nb1_counter_prev == nb1_counter)
	{
		nb1_missed_counter = nb1_missed_counter+1;
        if (nb1_missed_counter>1)
		{
			nb1_current = false;
			nb1_drive_counter = nb1_drive_counter+1;
		}
	}
	else if (nb1_counter_prev == nb1_counter-1)
	{
		nb1_missed_counter = 0;
		nb1_good = true;
		if(nb1_diff_prev==1)
		{
			nb1_current = true;
			nb1_drive_counter = 0;
		}
	}
	else
	{
		nb1_missed_counter = 0;
		nb1_current = false;
		nb1_good = true;
		nb1_drive_counter = nb1_drive_counter-1;
	}

    if (nb2_counter_prev == nb2_counter)
	{
		nb2_missed_counter = nb2_missed_counter+1;
        if (nb2_missed_counter>1)
		{
			nb2_current = false;
			nb2_drive_counter = nb2_drive_counter+1;
		}
	}
	else if (nb2_counter_prev == nb2_counter-1)
	{
		nb2_missed_counter = 0;
		nb2_good = true;
		if(nb2_diff_prev==1)
		{
			nb2_current = true;
			nb2_drive_counter = 0;
		}
	}
	else
	{
		nb2_missed_counter = 0;
		nb2_current = false;
		nb2_good = true;
		nb2_drive_counter = nb2_drive_counter-1;
	}

	if (nbS_counter_prev == nbS_counter)
	{
		nbS_missed_counter = nbS_missed_counter+1;
        if (nbS_missed_counter>1)
		{
			nbS_current = false;
			nbS_drive_counter = nbS_drive_counter+1;
		}
	}
	else if (nbS_counter_prev == nbS_counter-1)
	{
		nbS_missed_counter = 0;
		nbS_good = true;
		if(nbS_diff_prev==1)
		{
			nbS_current = true;
			nbS_drive_counter = 0;
		}
	}
	else
	{
		nbS_missed_counter = 0;
		nbS_current = false;
		nbS_good = true;
		nbS_drive_counter = nbS_drive_counter-1;
	}

	nb1_diff_prev = nb1_counter - nb1_counter_prev;
	nb2_diff_prev = nb2_counter - nb2_counter_prev;
	nbS_diff_prev = nbS_counter - nbS_counter_prev;
	nb1_counter_prev = nb1_counter;
	nb2_counter_prev = nb2_counter;
	nbS_counter_prev = nbS_counter;
	prev_time1 = time1;
	prev_time2 = time2;
	prev_timeS = timeS;
}
