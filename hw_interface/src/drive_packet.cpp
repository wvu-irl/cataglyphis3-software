/**
 * i7_to_nb1_packet.cpp
 * Implementation of methods for the WVU-NSRR Packet Structured Serial #4 Class 
 * for stuffing topic msg and debug print
 */
#include "drive_packet.h" 

Drive_Packet::Drive_Packet(buffer_RW_t buffer_RW)
{
/*
	reader = false;
	read_buffer = 0;
	rbuf_size = 0;
	writer = false;
	write_buffer = 0;
	wbuf_size =0;
*/
	memset(buf,0,sizeof(buf));
	switch (buffer_RW)
	{
		case pktread:
			reader = true;
			read_buffer = buf;
			rbuf_size = sizeof(pkt);
			break;
		case pktwrite:
			writer = true;
			write_buffer = buf;
			wbuf_size = sizeof(pkt);
			break;
		default:
			//ros::ROS_ERROR("buffer_RW_t type error on packet initialization");
			break;
	}
	H3_def = '4';
	pkt.H1 = H1_def;
	pkt.H2 = H2_def;
	pkt.H3 = H3_def;
}

void Drive_Packet::unpackMsg() 
{
    /**
     * Copy packet data into topic msg data structure for ROS publishing
     */
    msg.counter					= pkt.counter^0xff;
    msg.clock					= pkt.clock^0xffff;
    msg.motor_1_encoder_count	= pkt.encoder1^0xffff;
    msg.motor_2_encoder_count	= pkt.encoder2^0xffff;
    msg.motor_3_encoder_count	= pkt.encoder3^0xffff;
    msg.battery_volts			= (pkt.battery_volts ^ 0xff)/5.0;
    msg.motor1_amps				= ((pkt.motor1_amps ^ 0xff)-128)/12.0;
    msg.motor2_amps				= ((pkt.motor2_amps ^ 0xff)-128)/12.0;
    msg.motor3_amps				= ((pkt.motor3_amps ^ 0xff)-128)/12.0;
}

void Drive_Packet::packMsg(const messages::encoder_data::ConstPtr& msg) 
{
    /**
     * Copy topic msg data into packet data structure for writing to the port
     */
    pkt.counter         		= incrementCounter();
    pkt.clock					= msg->clock;
    pkt.encoder1				= msg->motor_1_encoder_count;
    pkt.encoder2				= msg->motor_2_encoder_count;
    pkt.encoder3				= msg->motor_3_encoder_count;
}

void Drive_Packet::publishMsg(ros::Publisher* pub_ptr)
{
	pub_ptr->publish(msg);
}


void Drive_Packet::subscribeMsg()
{
	sub = nh.subscribe<messages::encoder_data>("roboteq/drivemotorin/subscribeerror", 1, &Drive_Packet::packMsg, this);
}


void Drive_Packet::debug_print_packet_data(int res)
{
    // debug: print character buffer
    /*ros::ROS_DEBUG(":%s:%d\n", buf, res);

    // debug: print structure fields
      ros::ROS_DEBUG("H1 (char): %d, %c\n", pkt.H1, pkt.H1);
      ros::ROS_DEBUG("H2 (char): %d, %c\n", pkt.H2, pkt.H2);
      ros::ROS_DEBUG("H3 (uint8): %d, %c\n", pkt.H3, pkt.H3);
      ros::ROS_DEBUG("counter (uint16): %d, %c,  %X\n", pkt.counter, pkt.counter, pkt.counter);
      ros::ROS_DEBUG("time (uint16): %d, %c,  %X\n", pkt.time, pkt.time, pkt.time);
      ros::ROS_DEBUG("object (uint8): %d, %c,  %X\n", pkt.object, pkt.object, pkt.object);
      ros::ROS_DEBUG("object conf (uint8): %d, %c,  %X\n", pkt.object_conf, pkt.object_conf, pkt.object_conf);
      ros::ROS_DEBUG("position_x (int16): %d, %c,  %X\n", pkt.position_x, pkt.position_x, pkt.position_x);
      ros::ROS_DEBUG("position_y (int16): %d, %c,  %X\n", pkt.position_y, pkt.position_y, pkt.position_y);
      ros::ROS_DEBUG("position_x_conf (uint16): %d, %c,  %X\n", pkt.position_x_conf, pkt.position_x_conf, pkt.position_x_conf);
      ros::ROS_DEBUG("position_y_conf (uint16): %d, %c,  %X\n", pkt.position_y_conf, pkt.position_y_conf, pkt.position_y_conf);
      ros::ROS_DEBUG("theta (int16): %d, %c,  %X\n", pkt.theta, pkt.theta, pkt.theta);
      ros::ROS_DEBUG("phi (int16): %d, %c,  %X\n", pkt.phi, pkt.phi, pkt.phi);
      ros::ROS_DEBUG("theta_conf (uint8): %d, %c,  %X\n", pkt.theta_conf, pkt.theta_conf, pkt.theta_conf);
      ros::ROS_DEBUG("phi_conf (uint8): %d, %c,  %X\n", pkt.phi_conf, pkt.phi_conf, pkt.phi_conf);
      ros::ROS_DEBUG("Reserved (uint64): %d, %c,  %X\n", pkt.Reserved, pkt.Reserved, pkt.Reserved);
      ros::ROS_DEBUG("checksum (uint8): %d, %c,  %X\n", pkt.checksum, pkt.checksum, pkt.checksum);
      fflush(stdout);*/
}
