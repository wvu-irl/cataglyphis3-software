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
	ros::Subscriber subscriber_imu3;
	ros::NodeHandle node;
	
	//netburner1 imu callback function
	void getIMU1Callback(const messages::nb1_to_i7_msg::ConstPtr &msg)
	{
		ROS_INFO("imu_callback 1 \n");
		this->p1 = msg->rate_p*3.1419527/180; //radians
		this->q1 = msg->rate_q*3.1419527/180; //radians
		this->r1 = msg->rate_r*3.1419527/180; //radians
		this->ax1 = msg->acc_x; //G's
		this->ay1 = msg->acc_y; //G's
		this->az1 = msg->acc_z; //G's
		this->num_imus1=msg->num_imus; //number of imus on nb
		this->nb1_counter=msg->counter; //counter from nb
		this->call_counter1=this->call_counter1+1; //increments each time function executed
		this->time1=msg->nb_clock; //time since object instantiated
	}

	//netburner2 imu callback function	
	void getIMU2Callback(const messages::nb2_3_to_i7_msg::ConstPtr &msg)
	{
		this->p2 = msg->rate_p*3.1419527/180; //radians
		this->q2 = msg->rate_q*3.1419527/180; //radians
		this->r2 = msg->rate_r*3.1419527/180; //radians
		this->ax2 = msg->acc_x; //G's
		this->ay2 = msg->acc_y; //G's
		this->az2 = msg->acc_z; //G's
		this->num_imus2=msg->num_imus; //number of imus on nb
		this->nb2_counter=msg->counter; //counter from nb
		this->call_counter2=this->call_counter2+1; //increments each time function executed
		this->time2=msg->nb_clock; //time since object instantiated
	}

	//netburner3 imu callback function
	void getIMU3Callback(const messages::nb2_3_to_i7_msg::ConstPtr &msg)
	{
		this->p3 = msg->rate_p*3.1419527/180; //radians
		this->q3 = msg->rate_q*3.1419527/180; //radians
		this->r3 = msg->rate_r*3.1419527/180; //radians
		this->ax3 = msg->acc_x; //G's
		this->ay3 = msg->acc_y; //G's
		this->az3 = msg->acc_z; //G's
		this->num_imus3=msg->num_imus; //number of imus on nb
		this->nb3_counter=msg->counter; //counter from nb
		this->call_counter3=this->call_counter3+1; //increments each time function executed
		this->time3=msg->nb_clock; //time since object instantiated
	}
public:
	double start_time;
	double p, p1, p2, p3;
	double q, q1, q2, q3;
	double r, r1, r2, r3;
	double ax, ax1, ax2, ax3;
	double ay, ay1, ay2, ay3;
	double az, az1, az2, az3;
	double prev_time1, prev_time2, prev_time3;
	double dt1, dt2, dt3;
	short int call_counter1, call_counter2, call_counter3;
	short int nb1_counter, nb2_counter, nb3_counter;
	short int nb1_counter_prev, nb2_counter_prev, nb3_counter_prev;
	short int num_imus1, num_imus2, num_imus3;
	double time1, time2, time3;
	double p1_offset, q1_offset, r1_offset;
	double p2_offset, q2_offset, r2_offset;
	double p3_offset, q3_offset, r3_offset;
	double E_p1_offset, E_q1_offset, E_r1_offset;
	double E_p2_offset, E_q2_offset, E_r2_offset;
	double E_p3_offset, E_q3_offset, E_r3_offset;
	double mean_p1, mean_q1, mean_r1;
	double mean_p2, mean_q2, mean_r2;
	double mean_p3, mean_q3, mean_r3;
	double bias_thresh;
	bool good_bias1, good_bias2, good_bias3;
	short int new_imu, new_imu1, new_imu2, new_imu3;
	arma::mat p1_values;
	arma::mat q1_values;
	arma::mat r1_values;
	arma::mat p2_values;
	arma::mat q2_values;
	arma::mat r2_values;
	arma::mat p3_values;
	arma::mat q3_values;
	arma::mat r3_values;

	IMU();
	void collect_gyro1_data();
	void collect_gyro2_data();
	void collect_gyro3_data();
	void calculate_gyro1_offset();
	void calculate_gyro2_offset();
	void calculate_gyro3_offset();
	void set_gyro1_offset();
	void set_gyro2_offset();
	void set_gyro3_offset();
	void clear_gyro1_values();
	void clear_gyro2_values();
	void clear_gyro3_values();
	void filter_imu_values();
	void set_prev_counters();
	void determine_new_data();
};

#endif