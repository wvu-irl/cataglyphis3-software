#include <ros/ros.h>
//#include <roboteq_interface/encoder_data.h>
#include <armadillo>


#ifndef FILTER_CLASS_HPP
#define FILTER_CLASS_HPP

class Filter
{
private:

public:
	double phi;
	double theta;
	double psi;
	double x;
	double y;
	double x_prev;
	double y_prev;
	double x_prev_waiting;
	double y_prev_waiting;
	double pqr_stddev;
	double PI;
	double a_stddev;
	double P_x;
	double P_y;
	double P_phi;
	double P_theta;
	double P_psi;
	double d_thresh;
	double Q_delta_d;
	double Q_phi;
	double Q_theta;
	double Q_psi;
	double north_angle;
	double Kens_north_angle;
	double Kens_angle;
	double P_north_angle;
	double E_north_angle;
	double north_angle_thresh;
	double heading_update;
	int keep_nb;
	bool homing_verified;
	double b_h_diff_k; 
	double heading_est_k;
	double heading_est_k_prev;
	double heading_prev;
	double l_dist;
	short int platform_number;
	arma::mat p_values;
	arma::mat q_values;
	arma::mat r_values;
	arma::mat ax_values;
	arma::mat ay_values;
	arma::mat az_values;
	arma::mat dull_x_vec;
	arma::mat dull_y_vec;
	arma::mat shiny_x_vec;
	arma::mat shiny_y_vec;


	unsigned short int counter;
	Filter();
	void initialize_states(double phi_init, double theta_init, double psi_init, double x_init, double y_init, double P_phi_init, double P_theta_init, double P_psi_init, double P_x_init, double P_y_init);
	void initialize_variance(int performed_bias_removal, double north_angle_unc);
	void dead_reckoning(double p, double q, double r, double delta_distance, double dt);
	void turning(double p, double q, double r, double dt);
	void collect_accelerometer_data(double ax, double ay, double az);
	void roll_pitch_G_update();
	void clear_accelerometer_values();
	void blind_dead_reckoning(double p, double q, double r, double delta_distance, double dt);
	void blind_turning(double p, double q, double r, double dt);
	void which_nb_to_keep(int nb1_drive_counter, bool nb1_current, bool nb1_good, bool nb1_good_prev, int nb2_drive_counter, bool nb2_current, bool nb2_good, bool nb2_good_prev, bool nbS_current, bool nbS_good, bool nbS_good_prev);
	void homing_update(double homing_heading, double homing_x, double homing_y, double dull_x, double dull_y, double shiny_x, double shiny_y, double cylinder_std, bool possibly_lost, bool square_update);
	void clear_cylinder_vec();
	void find_Kens_north_angle();
	void check_Kens_north_angle();

};

#endif