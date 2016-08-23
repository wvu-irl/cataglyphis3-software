#include <ros/ros.h>
#include <messages/nb1_to_i7_msg.h>
#include <messages/nb2_3_to_i7_msg.h>
#include <armadillo>

#ifndef IMU_CLASS_HPP
#define IMU_CLASS_HPP

class IMU
{
private:
	ros::Subscriber subscriber_imu1;
	ros::Subscriber subscriber_imu2;
	ros::NodeHandle node;
	
	//netburner1 imu callback function
	void getIMU1Callback(const messages::nb1_to_i7_msg::ConstPtr &msg)
	{
		//ROS_INFO("imu_callback 1 \n");
		this->p1 = msg->rate_p1*3.1419527/180; //radians
		this->q1 = msg->rate_q1*3.1419527/180; //radians
		this->r1 = msg->rate_r1*3.1419527/180; //radians
		this->ax1 = msg->acc_x1; //G's
		this->ay1 = msg->acc_y1; //G's
		this->az1 = msg->acc_z1; //G's
		this->p2 = msg->rate_p2*3.1419527/180; //radians
		this->q2 = msg->rate_q2*3.1419527/180; //radians
		this->r2 = msg->rate_r2*3.1419527/180; //radians
		this->ax2 = msg->acc_x2; //G's
		this->ay2 = msg->acc_y2; //G's
		this->az2 = msg->acc_z2; //G's
		this->p3 = msg->rate_p3*3.1419527/180; //radians
		this->q3 = msg->rate_q3*3.1419527/180; //radians
		this->r3 = msg->rate_r3*3.1419527/180; //radians
		this->ax3 = msg->acc_x3; //G's
		this->ay3 = msg->acc_y3; //G's
		this->az3 = msg->acc_z3; //G's
		this->nb1_imu_nums=msg->num_imus; //number of imus on nb
		this->nb1_counter=msg->counter; //counter from nb
		this->call_counter1=this->call_counter1+1; //increments each time function executed
		this->time1=msg->nb_clock; //time since object instantiated
	}

	//netburner2 imu callback function	
	void getIMU2Callback(const messages::nb2_3_to_i7_msg::ConstPtr &msg)
	{
		this->p4 = msg->rate_p1*3.1419527/180; //radians
		this->q4 = msg->rate_q1*3.1419527/180; //radians
		this->r4 = msg->rate_r1*3.1419527/180; //radians
		this->ax4 = msg->acc_x1; //G's
		this->ay4 = msg->acc_y1; //G's
		this->az4 = msg->acc_z1; //G's
		this->p5 = msg->rate_p2*3.1419527/180; //radians
		this->q5 = msg->rate_q2*3.1419527/180; //radians
		this->r5 = msg->rate_r2*3.1419527/180; //radians
		this->ax5 = msg->acc_x2; //G's
		this->ay5 = msg->acc_y2; //G's
		this->az5 = msg->acc_z2; //G's
		this->p6 = msg->rate_p3*3.1419527/180; //radians
		this->q6 = msg->rate_q3*3.1419527/180; //radians
		this->r6 = msg->rate_r3*3.1419527/180; //radians
		this->ax6 = msg->acc_x3; //G's
		this->ay6 = msg->acc_y3; //G's
		this->az6 = msg->acc_z3; //G's
		this->nb2_imu_nums=msg->num_imus; //number of imus on nb
		this->nb2_counter=msg->counter; //counter from nb
		this->call_counter2=this->call_counter2+1; //increments each time function executed
		this->time2=msg->nb_clock; //time since object instantiated
	}


public:
	double start_time;
	double p, p1, p2, p3, p4, p5, p6, pS;
	double q, q1, q2, q3, q4, q5, q6, qS;
	double r, r1, r2, r3, r4, r5, r6, rS;
	double ax, ax1, ax2, ax3, ax4, ax5, ax6, axS;
	double ay, ay1, ay2, ay3, ay4, ay5, ay6, ayS;
	double az, az1, az2, az3, az4, az5, az6, azS;

	double nb1_p, nb2_p, nbS_p;
	double nb1_q, nb2_q, nbS_q;
	double nb1_r, nb2_r, nbS_r;
	double nb1_ax, nb2_ax, nbS_ax;
	double nb1_ay, nb2_ay, nbS_ay;
	double nb1_az, nb2_az, nbS_az;

	double prev_time1, prev_time2, prev_timeS;
	double dt1, dt2, dtS;
	short int call_counter1, call_counter2, call_counterS;
	short int nb1_counter, nb2_counter, nbS_counter;
	short int nb1_counter_prev, nb2_counter_prev, nbS_counter_prev;
	short int nb1_imu_nums, nb2_imu_nums;
	short int nb1_num_imus, nb2_num_imus;
	double time1, time2, timeS;

	double p1_offset, q1_offset, r1_offset;
	double p2_offset, q2_offset, r2_offset;
	double p3_offset, q3_offset, r3_offset;
	double p4_offset, q4_offset, r4_offset;
	double p5_offset, q5_offset, r5_offset;
	double p6_offset, q6_offset, r6_offset;
	double pS_offset, qS_offset, rS_offset;

	double E_p1_offset, E_q1_offset, E_r1_offset;
	double E_p2_offset, E_q2_offset, E_r2_offset;
	double E_p3_offset, E_q3_offset, E_r3_offset;
	double E_p4_offset, E_q4_offset, E_r4_offset;
	double E_p5_offset, E_q5_offset, E_r5_offset;
	double E_p6_offset, E_q6_offset, E_r6_offset;
	double E_pS_offset, E_qS_offset, E_rS_offset;

	double mean_p1, mean_q1, mean_r1;
	double mean_p2, mean_q2, mean_r2;
	double mean_p3, mean_q3, mean_r3;
	double mean_p4, mean_q4, mean_r4;
	double mean_p5, mean_q5, mean_r5;
	double mean_p6, mean_q6, mean_r6;
	double mean_pS, mean_qS, mean_rS;

	double bias_thresh;
	bool good_bias1, good_bias2, good_bias3, good_bias4, good_bias5, good_bias6, good_biasS;
	short int new_imu1, new_imu2, new_imu3, new_imu4, new_imu5, new_imu6, new_imuS;

	short int new_nb1, new_nb2, new_nbS;
	int nb1_missed_counter, nb2_missed_counter, nbS_missed_counter;
	int nb1_drive_counter, nb2_drive_counter, nbS_drive_counter;
	int nb1_diff_prev, nb2_diff_prev, nbS_diff_prev;
	bool nb1_current, nb2_current, nbS_current;
	bool nb1_good, nb2_good, nbS_good;
	bool nb1_good_prev, nb2_good_prev, nbS_good_prev;

	arma::mat p1_values;
	arma::mat q1_values;
	arma::mat r1_values;
	arma::mat p2_values;
	arma::mat q2_values;
	arma::mat r2_values;
	arma::mat p3_values;
	arma::mat q3_values;
	arma::mat r3_values;
	arma::mat p4_values;
	arma::mat q4_values;
	arma::mat r4_values;
	arma::mat p5_values;
	arma::mat q5_values;
	arma::mat r5_values;
	arma::mat p6_values;
	arma::mat q6_values;
	arma::mat r6_values;
	arma::mat pS_values;
	arma::mat qS_values;
	arma::mat rS_values;

	IMU();
	void collect_gyro1_data();
	void collect_gyro2_data();
	void collect_gyro3_data();
	void collect_gyro4_data();
	void collect_gyro5_data();
	void collect_gyro6_data();
	void collect_gyroS_data();

	void calculate_gyro1_offset();
	void calculate_gyro2_offset();
	void calculate_gyro3_offset();
	void calculate_gyro4_offset();
	void calculate_gyro5_offset();
	void calculate_gyro6_offset();
	void calculate_gyroS_offset();

	void set_gyro1_offset();
	void set_gyro2_offset();
	void set_gyro3_offset();
	void set_gyro4_offset();
	void set_gyro5_offset();
	void set_gyro6_offset();
	void set_gyroS_offset();

	void clear_gyro1_values();
	void clear_gyro2_values();
	void clear_gyro3_values();
	void clear_gyro4_values();
	void clear_gyro5_values();
	void clear_gyro6_values();
	void clear_gyroS_values();

	void clear_gyro_values();
	void filter_imu_values();
	void set_prev_counters();
	void determine_new_data();
};

#endif
