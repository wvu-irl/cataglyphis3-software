/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

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
