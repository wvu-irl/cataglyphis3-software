#include "navigation/filter_class.hpp"

Filter::Filter()
{
	phi=0;
	theta=0;
	psi=0;
	x=0;
	y=0;
	PI = 3.1415927;
	counter=0;
	pqr_stddev = 0.01;
	a_stddev = 0.05;
	d_thresh = 8.0;
	ax_values.clear();
	ay_values.clear();
	az_values.clear();
	P_x = 1.0;
	P_y = 1.0;
	P_phi = 0.05;
	P_theta = 0.05;
	P_psi = 0.05;
	Q_delta_d = 0.001;
	Q_phi = 2.2847e-008;
	Q_theta = 2.2847e-008;
	Q_psi = 2.2847e-008;
	north_angle = 0.0;
	P_north_angle = 25.0;
	keep_nb = 3;
	heading_verified = false;
	E_north_angle = 111.0*PI/180.0;
	north_angle_thresh = 75.0*PI/180.0;
}

void Filter::initialize_states(double phi_init, double theta_init, double psi_init, double x_init, double y_init, double P_phi_init, double P_theta_init, double P_psi_init, double P_x_init, double P_y_init)
{
	phi = phi_init;
	theta = theta_init;
	psi = psi_init;
	x = x_init;
	y = y_init;
	P_phi = P_phi_init;
	P_theta = P_theta_init;
	P_psi = P_psi_init;
	P_x = P_x_init;
	P_y = P_y_init;
}

void Filter::initialize_variance(int performed_bias_removal, double north_angle_unc)
{
	if(performed_bias_removal != 1)
	{
		Q_phi = 0.0000036;
		Q_theta = 0.0000036;
		Q_psi = 0.0000036;
	}
	P_psi = north_angle_unc*north_angle_unc;
		
}

void Filter::which_nb_to_keep(int nb1_drive_counter, bool nb1_current, int nb2_drive_counter, bool nb2_current, bool nb1_good_prev, bool nb2_good_prev)
{
	if (nb1_current && nb2_current)
	{
		keep_nb = 3;
	}
	else if (nb1_current && !nb2_current && nb1_good_prev)
	{
		keep_nb = 1;
	}
	else if (!nb1_current && nb2_current && nb2_good_prev)
	{
		keep_nb = 2;
	}
	else
	{
		if (nb1_drive_counter <= nb2_drive_counter)
		{
			if(nb1_good_prev)
			{
				keep_nb = 1;
			}
			else if (nb2_good_prev)
			{
				keep_nb = 2;
			}
			else
			{
				keep_nb = 1;
			}
		}
		else
		{
			if(nb2_good_prev)
			{
				keep_nb = 2;
			}
			else if (nb1_good_prev)
			{
				keep_nb = 1;
			}
			else
			{
				keep_nb = 2;
			}
		}
	}
}

void Filter::collect_accelerometer_data(double ax, double ay, double az)
{
	ax_values = arma::join_vert(ax_values,arma::mat(1,1,arma::fill::ones)*ax);
	ay_values = arma::join_vert(ay_values,arma::mat(1,1,arma::fill::ones)*ay);
	az_values = arma::join_vert(az_values,arma::mat(1,1,arma::fill::ones)*az);
}


void Filter::dead_reckoning(double p, double q, double r, double delta_distance, double dt)
{
	double phi_m = phi;
	double theta_m = theta;
	double psi_m = psi;
	phi = phi_m+dt*(p+(q)*sin(phi_m)*tan(theta_m)+(r)*cos(phi_m)*tan(theta_m));
	theta = theta_m+dt*((q)*cos(phi_m)-(r)*sin(phi_m));
	psi = psi_m+dt*((q)*sin(phi_m)+(r)*cos(phi_m))*(1/cos(theta_m));
	x = x+delta_distance*cos(psi_m)*cos(theta_m);
	y = y+delta_distance*sin(psi_m)*cos(theta_m);

	Q_phi = Q_phi+2.2847e-012;
	Q_theta = Q_theta+2.2847e-012;
	Q_psi = Q_psi+2.2847e-012;
	P_x = P_x+Q_delta_d*cos(psi)+Q_delta_d*sin(sqrt(P_psi));
	P_y = P_x+Q_delta_d*sin(psi)+Q_delta_d*sin(sqrt(P_psi));;
	P_phi = P_phi+Q_phi;
	P_theta = P_theta+Q_theta;
	P_psi = P_psi+Q_psi;
}

void Filter::turning(double p, double q, double r, double dt)
{
	double phi_m = phi;
	double theta_m = theta;
	double psi_m = psi;
	phi = phi_m+dt*(p+(q)*sin(phi_m)*tan(theta_m)+(r)*cos(phi_m)*tan(theta_m));
	theta = theta_m+dt*((q)*cos(phi_m)-(r)*sin(phi_m));
	psi = psi_m+dt*((q)*sin(phi_m)+(r)*cos(phi_m))*(1/cos(theta_m));
	
	Q_phi = Q_phi+2.2847e-012;
	Q_theta = Q_theta+2.2847e-012;
	Q_psi = Q_psi+2.2847e-012;
	P_phi = P_phi+Q_phi;
	P_theta = P_theta+Q_theta;
	P_psi = P_psi+Q_psi;
}


void Filter::clear_accelerometer_values()
{
	ax_values.clear();
	ay_values.clear();
	az_values.clear();
}


void Filter::roll_pitch_G_update()
{
	arma::mat ax_holder1 = ax_values;
	arma::mat ay_holder1 = ay_values;
	arma::mat az_holder1 = az_values;
	arma::mat ax_holder2;
	arma::mat ay_holder2;
	arma::mat az_holder2;
	arma::mat ax_meds;
	arma::mat ay_meds;
	arma::mat az_meds;
	arma::mat m_temp;

	double ax_est = 0;
	double ay_est = 0;
	double az_est = 0;


	// ax_est
	double m_ax = 0;
	int s_ax = 0;
	double min_dist_ax = 0;
	int min_index_ax = 0;
	ax_holder2.clear();
	ax_meds.clear();
	for (int jj = 0; jj<10; jj++)
	{
		m_temp = arma::median(arma::median(ax_holder1));
		m_ax = m_temp(0,0);
		s_ax = ax_holder1.size();
		min_dist_ax = fabs(ax_holder1(0,0)-m_ax);
		min_index_ax = 0;
		for (int ii = 1; ii<s_ax; ii++)
		{
			if (fabs(ax_holder1(ii,0)-m_ax) < min_dist_ax)
			{
				min_dist_ax = fabs(ax_holder1(ii,0)-m_ax);
				min_index_ax = ii;
			}
		}
		
		for (int ii = 0; ii<s_ax; ii++)
		{
			if (ii != min_index_ax)
			{
				ax_holder2 = arma::join_vert(ax_holder2,arma::mat(1,1,arma::fill::ones)*ax_holder1(ii,0));
			}
			else
			{
				ax_meds = arma::join_vert(ax_meds,arma::mat(1,1,arma::fill::ones)*ax_holder1(ii,0));
			}
		}
		ax_holder1.clear();
		ax_holder1 = ax_holder2;
		ax_holder2.clear();
	}
	ax_est = arma::mean(arma::mean(ax_meds));


	// ay_est
	double m_ay = 0;
	int s_ay = 0;
	double min_dist_ay = 0;
	int min_index_ay = 0;
	ay_holder2.clear();
	ay_meds.clear();
	for (int jj = 0; jj<10; jj++)
	{
		m_temp = arma::median(arma::median(ay_holder1));
		m_ay = m_temp(0,0);
		s_ay = ay_holder1.size();
		min_dist_ay = fabs(ay_holder1(0,0)-m_ay);
		min_index_ay = 0;
		for (int ii = 1; ii<s_ay; ii++)
		{
			if (fabs(ay_holder1(ii,0)-m_ay) < min_dist_ay)
			{
				min_dist_ay = fabs(ay_holder1(ii,0)-m_ay);
				min_index_ay = ii;
			}
		}
		
		for (int ii = 0; ii<s_ay; ii++)
		{
			if (ii != min_index_ay)
			{
				ay_holder2 = arma::join_vert(ay_holder2,arma::mat(1,1,arma::fill::ones)*ay_holder1(ii,0));
			}
			else
			{
				ay_meds = arma::join_vert(ay_meds,arma::mat(1,1,arma::fill::ones)*ay_holder1(ii,0));
			}
		}
		ay_holder1.clear();
		ay_holder1 = ay_holder2;
		ay_holder2.clear();
	}
	ay_est = arma::mean(arma::mean(ay_meds));


	// az_est
	double m_az = 0;
	int s_az = 0;
	double min_dist_az = 0;
	int min_index_az = 0;
	az_holder2.clear();
	az_meds.clear();
	for (int jj = 0; jj<10; jj++)
	{
		m_temp = arma::median(arma::median(az_holder1));
		m_az = m_temp(0,0);
		s_az = az_holder1.size();
		min_dist_az = fabs(az_holder1(0,0)-m_az);
		min_index_az = 0;
		for (int ii = 1; ii<s_az; ii++)
		{
			if (fabs(az_holder1(ii,0)-m_az) < min_dist_az)
			{
				min_dist_az = fabs(az_holder1(ii,0)-m_az);
				min_index_az = ii;
			}
		}
		
		for (int ii = 0; ii<s_az; ii++)
		{
			if (ii != min_index_az)
			{
				az_holder2 = arma::join_vert(az_holder2,arma::mat(1,1,arma::fill::ones)*az_holder1(ii,0));
			}
			else
			{
				az_meds = arma::join_vert(az_meds,arma::mat(1,1,arma::fill::ones)*az_holder1(ii,0));
			}
		}
		az_holder1.clear();
		az_holder1 = az_holder2;
		az_holder2.clear();
	}
	az_est = arma::mean(arma::mean(az_meds));

	/*
	arma::mat m_temp = arma::median(arma::median(ax_values));
	double ax_est = m_temp(0,0);
	m_temp = arma::median(arma::median(ay_values));
	double ay_est = m_temp(0,0);
	m_temp = arma::median(arma::median(az_values));
	double az_est = m_temp(0,0);*/

	if (ax_est!=0 && ay_est!=0 && az_est!=0)
	{
		arma::mat X(2,1);
		arma::mat F(3,1);
		arma::mat J(3,2);
		arma::mat G_vect(3,1);
		G_vect(0,0) = 0.0;
		G_vect(1,0) = 0.0;
		G_vect(2,0) = -1.0;
		X(0,0) = 0;
		X(1,0) = 0;
		double phi_est;
		double theta_est;
		for(int ii = 0;ii<20;ii++)
		{
			phi_est = X(0,0);
			theta_est = X(1,0);
			
			F(0,0) = cos(theta_est)*ax_est+sin(theta_est)*sin(phi_est)*ay_est+sin(theta_est)*cos(phi_est)*az_est;
			F(1,0) = cos(phi_est)*ay_est-sin(phi_est)*az_est;
			F(2,0) = -sin(theta_est)*ax_est+cos(theta_est)*sin(phi_est)*ay_est+cos(theta_est)*cos(phi_est)*az_est;

			J(0,0) = ay_est*cos(phi_est)*sin(theta_est) - az_est*sin(phi_est)*sin(theta_est);
			J(1,0) = -az_est*cos(phi_est) - ay_est*sin(phi_est);
			J(2,0) = ay_est*cos(phi_est)*cos(theta_est) - az_est*cos(theta_est)*sin(phi_est);
			J(0,1) = az_est*cos(phi_est)*cos(theta_est) - ax_est*sin(theta_est) + ay_est*cos(theta_est)*sin(phi_est);
			J(1,1) = 0;
			J(2,1) = -ax_est*cos(theta_est) - az_est*cos(phi_est)*sin(theta_est) - ay_est*sin(phi_est)*sin(theta_est);
			X = X-arma::inv(J.st()*J)*J.st()*(F-G_vect);
		}
		phi = atan(tan(X(0,0)));
		theta = atan(tan(X(1,0)));
		P_phi = 0.3;
		P_theta = 0.3;
	}
}

void Filter::blind_dead_reckoning(double p, double q, double r, double delta_distance, double dt)
{
	x = x+delta_distance*cos(psi)*cos(theta);
	y = y+delta_distance*sin(psi)*cos(theta);

	Q_phi = Q_phi+2.2847e-012;
	Q_theta = Q_theta+2.2847e-012;
	Q_psi = Q_psi+2.2847e-012;
	P_x = P_x+Q_delta_d*cos(psi)+Q_delta_d*sin(sqrt(P_psi));
	P_y = P_x+Q_delta_d*sin(psi)+Q_delta_d*sin(sqrt(P_psi));;
	P_phi = P_phi+p*p*dt*dt;
	P_theta = P_theta+q*q*dt*dt;
	P_psi = P_psi+r*r*dt*dt;
}

void Filter::blind_turning(double p, double q, double r, double dt)
{
	Q_phi = Q_phi+2.2847e-012;
	Q_theta = Q_theta+2.2847e-012;
	Q_psi = Q_psi+2.2847e-012;
	P_phi = P_phi+p*p*dt*dt;
	P_theta = P_theta+q*q*dt*dt;
	P_psi = P_psi+r*r*dt*dt;
}

void Filter::verify_homing(double homing_heading, double homing_x, double homing_y)
{
	if (homing_heading<psi)
	{
		while (homing_heading<psi)
		{
			homing_heading = homing_heading+2*PI;
		}
		if (fabs(homing_heading-psi)>fabs((homing_heading-2*PI)-psi))
		{
			homing_heading = homing_heading-2*PI;
		}
	}
	if (homing_heading>psi)
	{
		while (homing_heading>psi)
		{
			homing_heading = homing_heading-2*PI;
		}
		if (fabs(homing_heading-psi)>fabs((homing_heading+2*PI)-psi))
		{
			homing_heading = homing_heading+2*PI;
		}
	}
	if(fabs(homing_heading-psi)<5.0*PI/180.0)
	{
		heading_verified = true;
	}
	else
	{
		heading_verified = false;
	}
	if(sqrt((homing_x-x)*(homing_x-x)+(homing_y-y)*(homing_y-y))>30.0)
	{
		heading_verified = false;
	}

}