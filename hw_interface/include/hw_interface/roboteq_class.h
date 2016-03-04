#ifndef ROBOTEQ_CLASS_H
#define ROBOTEQ_CLASS_H

#include <ros/ros.h>
#include <Buffer_Interface_class.h>
#include <Serial_Port_class.h>
#include <messages/ActuatorOut.h>
#include <messages/encoder_data.h>
#include <messages/GrabberFeedback.h>
#include <drive_packet.h>
#include <grabber_packet.h>
#include <string>
#include <boost/lexical_cast.hpp>

class Roboteq_Class : public Buffer_Interface
{
public:
	// Members
	// Drive roboteq fields
	int motor_1_speed_cmd;
	int motor_2_speed_cmd;
	int motor_3_speed_cmd;
	int motor_1_encoder_count;
	int motor_2_encoder_count;
	int motor_3_encoder_count;
	int motor_1_encoder_count_prev;
	int motor_2_encoder_count_prev;
	int motor_3_encoder_count_prev;
	messages::encoder_data encoder_msg_out;
	// Grabber roboteq fields
	int slide_pos_cmd = 1000;
	int drop_pos_cmd = -1000;
	int slide_pos_cmd_prev = 1000;
	int drop_pos_cmd_prev = -1000;
	int grabber_stop_cmd = 0;
	int slide_status;
	int drop_status;
	int error_status;
	int slide_pos_status;
	int counter;
	int checksum;
	int computed_checksum;
	messages::GrabberFeedback grabber_msg_out;
	// Other fields
	int num_channels = 2;
	double callback_time;
	double left_encoder_time;
	double right_encoder_time;
	const float inter_command_delay = 0.001;
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub;
	Serial_Port* port_ptr;
	char write_buffer_local[100];
	char read_buffer_local[100];
	Packet* packet_in_ptr;
	
	// Methods
	Roboteq_Class(); // constructor
	~Roboteq_Class(); // destructor
	void clearPluses();
	// Callbacks
	void frontCallback(const messages::ActuatorOut::ConstPtr& msg_in);
	void middleCallback(const messages::ActuatorOut::ConstPtr& msg_in);
	void backCallback(const messages::ActuatorOut::ConstPtr& msg_in);
	void leftCallback(const messages::ActuatorOut::ConstPtr& msg_in);
	void rightCallback(const messages::ActuatorOut::ConstPtr& msg_in);
	void grabberCallback(const messages::ActuatorOut::ConstPtr& msg_in);
	// Drive roboteq methods
	void setMotorSpeeds();
	void readEncoderValues();
	void setPrevValues();
	// Grabber roboteq methods
	void setGrabberCommands();
	void grabberStop();
	void readGrabberFeedback();
};

#endif /* ROBOTEQ_CLASS_H */
