#include "Comm_class.h"

Comm::Comm()
{
	ROS_INFO(" - - - Comm constructor start");
	if(ros::param::get("node_type", node_type)==false) 
	{
		ROS_ERROR("Comm node_type parameter not specified properly"); 
		node_type = "i7_nb1_sender";
	}
	if(ros::param::get("write_rate", write_rate)==false) 
	{
		ROS_WARN("Comm write_rate parameter not specified properly, set to 50 Hz by default"); 
		write_rate = 50;
//		write_rate = 1;   //~~~real slow for testing
	}
	ROS_INFO(" - - - ros::param complete");
	loop_rate_ptr = new ros::Rate(write_rate);
	ROS_INFO(" - - - ros::Rate set");
	if(node_type=="i7_nb1_serial")
	{
		ROS_INFO(" - - - - node_type==i7_nb1_serial");
		packet_in_ptr = new NB1_To_I7(pktread);
		ROS_INFO(" - - - - new NB1_To_I7(pktread)");
		packet_out_ptr = new I7_To_NB1(pktwrite);
		ROS_INFO(" - - - - new I7_To_NB1(pktwrite)");
		port_ptr = new Serial_Port(_fixed_length_packet);
		ROS_INFO(" - - - - new Serial_Port");
		pub = nh.advertise<messages::nb1_to_i7_msg>("hw_interface/nb1in/nb1in",1);
		ROS_INFO(" - - - - advertise<hw_interface::nb1_to_i7_msg>(hw_interface/nb1in/nb1in,1)");
		packet_out_ptr->subscribeMsg();		
		ROS_INFO(" - - - - packet_out_ptr->subscribeMsg()");
		publish_enabled = false;
		//Serial_act_ptr = new Serial_HSM_Act(&publish_enabled);
	}
	else if(node_type=="i7_nb1_udp_sender")
	{
		ROS_INFO(" - - - - node_type==i7_nb1_sender");
		packet_out_ptr = new I7_To_NB1(pktwrite);
		ROS_INFO(" - - - - new I7_To_NB1(pktwrite)");
		port_ptr = new UDP_Sender_Port;
		ROS_INFO(" - - - - new UDP_Sender_Port");
		packet_out_ptr->subscribeMsg();		
		ROS_INFO(" - - - - packet_out_ptr->subscribeMsg()");
	}
	else if(node_type=="nb1_i7_udp_receiver")
	{
		ROS_INFO(" - - - - node_type==nb1_i7_udp_receiver");
		packet_in_ptr = new NB1_To_I7(pktread);
		ROS_INFO(" - - - - new NB1_To_I7(pktread)");
		port_ptr = new UDP_Receiver_Port;
		ROS_INFO(" - - - - new UDP_Sender_Port");
		pub = nh.advertise<messages::nb1_to_i7_msg>("hw_interface/nb1in/nb1in",1);
		ROS_INFO(" - - - - advertise<hw_interface::nb1_to_i7_msg>(hw_interface/nb1in/nb1in,1)");
		publish_enabled = true;
		//UDP_act_ptr	 = new UDP_HSM_Act(&publish_enabled);
	}
	else if(node_type=="nb2_i7_udp_receiver")
	{
		ROS_INFO(" - - - - node_type==nb2_i7_udp_receiver");
		packet_in_ptr = new NB2_3_To_I7(pktread);
		ROS_INFO(" - - - - new NB2_3_To_I7(pktread)");
		port_ptr = new UDP_Receiver_Port;
		ROS_INFO(" - - - - new UDP_Sender_Port");
		pub = nh.advertise<messages::nb2_3_to_i7_msg>("hw_interface/nb2in/nb2in",1);
		ROS_INFO(" - - - - advertise<hw_interface::nb2_3_to_i7_msg>(commhw_interfacenb2in/nb2in,1)");
		publish_enabled = true;
		//UDP_act_ptr	 = new UDP_HSM_Act(&publish_enabled);
	}
	else if(node_type=="nb3_i7_udp_receiver")
	{
		ROS_INFO(" - - - - node_type==nb3_i7_udp_receiver");
		packet_in_ptr = new NB2_3_To_I7(pktread);
		ROS_INFO(" - - - - new NB2_3_To_I7(pktread)");
		port_ptr = new UDP_Receiver_Port;
		ROS_INFO(" - - - - new UDP_Sender_Port");
		pub = nh.advertise<messages::nb2_3_to_i7_msg>("hw_interface/nb3in/nb3in",1);
		ROS_INFO(" - - - - advertise<hw_interface::nb2_3_to_i7_msg>(hw_interface/nb3in/nb3in,1)");
		publish_enabled = true;
		//UDP_act_ptr	 = new UDP_HSM_Act(&publish_enabled);
	}
	else if(node_type=="pathfinder_nb2_serial")
	{
		/*
		ROS_INFO(" - - - - node_type==pathfinder_nb2_serial");
		packet_in_ptr = new Packet_4(pktread);
		ROS_INFO(" - - - - new Packet_4(pktread)");
		packet_out_ptr = new Packet_4(pktwrite);
		ROS_INFO(" - - - - new Packet_4(pktwrite)");
		port_ptr = new Serial_Port(_fixed_length_packet);
		ROS_INFO(" - - - - new Serial_Port");
		pub = nh.advertise<comm::pkt_4_msg>("nb2_in_data",1);
		ROS_INFO(" - - - - advertise<comm::pkt_4_msg>(nb2_in_data,1)");
		packet_out_ptr->subscribeMsg();		
		ROS_INFO(" - - - - packet_out_ptr->subscribeMsg()");
		publish_enabled = true;
		Serial_act_ptr = new Serial_HSM_Act(&publish_enabled);
		*/
	}
	/*else if(node_type=="ranging_radio_serial")
	{
		packet_in_ptr = new Packet_p2(pktread);
		packet_out_ptr = new Packet_p1(pktwrite);
		port_ptr = new Serial_Port;
		pub = nh.advertise<comm::Message_p2>("rr_in_data",1);
		sub = nh.subscribe<comm::Message_p1>("rr_out_data",1,packet_out_ptr->unpackMsg());
	}
	else if(node_type=="servos_serial")
	{
		packet_in_ptr = new Packet_p4(pktread);
		packet_out_ptr = new Packet_p3(pktwrite);
		port_ptr = new Serial_Port;
		pub = nh.advertise<comm::Message_p4>("servos_in_data",1);
		sub = nh.subscribe<comm::Message_p3>("servos_out_data",1,packet_out_ptr->unpackMsg());
	}*/
	else
	{
		ROS_ERROR("Comm node type parameter specified does not exist");
		// Do something here to deal with bad node type
	}
}

void Comm::run()
{
	//	double current_time = ros::Time::now().toSec(); 
	//	static double previous_time = current_time;
	//	double delta_time;
	ROS_INFO(" - - - - - in Comm::run()");
	if (reader) // Only execute reading and publishing if node is a reader
	{
		ROS_INFO(" - - - - - reader branch");
		if (port_ptr->port_read())
		{
			ROS_INFO(" - - - - - port_read() is true");
			packet_in_ptr->unpackMsg();
			if(publish_enabled) {ROS_INFO("*_*_*_*_ publish enabled"); packet_in_ptr->publishMsg(&pub);}  
		}
	}
	if (writer) // Only execute writing if node is a writer ~~~ and its time to write
	{
		ROS_INFO(" - - - - - writer branch");
		port_ptr->port_write();
	}
	ros::spinOnce();
	loop_rate_ptr->sleep(); // ~~~ Fix write rate structure so that write operates on a schedule and doesn't choke read
}
