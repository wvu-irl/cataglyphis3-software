#ifndef COMM_CLASS_H
#define COMM_CLASS_H

#include <ros/ros.h>
#include <string>

// Port includes
#include "Port_Superclass.h"
#include "Serial_Port_class.h"
#include "UDP_Sender_Port_class.h"
#include "UDP_Sender_Port_class_fault_test.h"
#include "UDP_Receiver_Port_class.h"

// Packet includes
#include "Buffer_Interface_class.h"
#include "Packet_Superclass.h"

#include "nb1_to_i7_packet.h"
#include "nb1_to_i7_serial_packet.h"
#include "i7_to_nb1_packet.h"
#include "nb2_3_to_i7_packet.h"

// Message includes

#include <messages/nb1_to_i7_msg.h>
#include <messages/nb1_to_i7_serial_msg.h>
#include <messages/i7_to_nb1_msg.h>
#include <messages/nb2_3_to_i7_msg.h>

// HSM includes
//#include "UDP_HSM_Act_class.h"
//#include "Serial_HSM_Act_class.h"

class Comm: public Buffer_Interface
{
public:
	// Members
	// ROS
	ros::NodeHandle nh;
	ros::Rate* loop_rate_ptr;
	ros::Publisher pub;
	Packet* packet_in_ptr;
	Packet* packet_out_ptr;
	Port* port_ptr;
	std::string node_type;
	//UDP_HSM_Act* UDP_act_ptr;
	//Serial_HSM_Act* Serial_act_ptr;
	int write_rate;
	bool publish_enabled;

	// Methods
	Comm(); // Constructor
	void run();
};

#endif /* COMM_CLASS_H */
