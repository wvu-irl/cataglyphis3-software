/**
 * nb2_3_to_i7_packet.cpp
 * Implementation of methods for the WVU-NSRR Packet Structured Serial #3 Class 
 * for stuffing topic msg and debug print
 */
#include "nb2_3_to_i7_packet.h" 

NB2_3_To_I7::NB2_3_To_I7(buffer_RW_t buffer_RW)
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
	H3_def = '3';
	pkt.H1 = H1_def;
	pkt.H2 = H2_def;
	pkt.H3 = H3_def;
}

void NB2_3_To_I7::unpackMsg() 
{
    /**
     * Copy packet data into topic msg data structure for ROS publishing
     */
    msg.counter         	= pkt.counter;
    msg.nb_clock			= ((double)pkt.clock_reg_count + ((double)pkt.clock_reg_reset_count * ((double) 0xFFFFFFFF + 1.0))) / (double)125000000.0;
    msg.acc_x1 			= pkt.acc_x1*0.00025/65536.0;
    msg.acc_y1 			= pkt.acc_y1*0.00025/65536.0;
    msg.acc_z1 			= pkt.acc_z1*0.00025/65536.0;
    msg.rate_p1          	= pkt.rate_p1*0.02/65536.0;
    msg.rate_q1          	= pkt.rate_q1*0.02/65536.0;
    msg.rate_r1          	= pkt.rate_r1*0.02/65536.0;
    
    msg.acc_x2 			= pkt.acc_x2*0.00025/65536.0;
    msg.acc_y2 			= pkt.acc_y2*0.00025/65536.0;
    msg.acc_z2 			= pkt.acc_z2*0.00025/65536.0;
    msg.rate_p2          	= pkt.rate_p2*0.02/65536.0;
    msg.rate_q2          	= pkt.rate_q2*0.02/65536.0;
    msg.rate_r2          	= pkt.rate_r2*0.02/65536.0;
    
    msg.acc_x3 			= pkt.acc_x3*0.00025/65536.0;
    msg.acc_y3 			= pkt.acc_y3*0.00025/65536.0;
    msg.acc_z3			= pkt.acc_z3*0.00025/65536.0;
    msg.rate_p3          	= pkt.rate_p3*0.02/65536.0;
    msg.rate_q3          	= pkt.rate_q3*0.02/65536.0;
    msg.rate_r3          	= pkt.rate_r3*0.02/65536.0;
    
    msg.num_imus			= pkt.num_imus;
    msg.pause_switch		= pkt.pause_switch;
    msg.main_loop_counter	= pkt.main_loop_counter;
    msg.i7_clock            = ros::Time::now().toSec();
    
    //msg.pot1                = (pkt.potValues[0] >> 3) / 8;
    //msg.pot2                = (pkt.potValues[1] >> 3) / 8;
    //msg.pot3                = (pkt.potValues[2] >> 3) / 8;
    //msg.pot4                = (pkt.potValues[3] >> 3) / 8;
    //msg.pot5                = (pkt.potValues[4] >> 3) / 8;
    //msg.pot6                = (pkt.potValues[5] >> 3) / 8;
    //msg.pot7                = (pkt.potValues[6] >> 3) / 8;
    //msg.pot8                = (pkt.potValues[7] >> 3) / 8;
    
    /*msg.xda                 = pkt.xda;
    msg.yda                 = pkt.yda;
    msg.zda                 = pkt.zda;
    msg.xdv                 = pkt.xdv;
    msg.ydv                 = pkt.ydv;
    msg.zdv                 = pkt.zdv;*/
}

void NB2_3_To_I7::packMsg(const messages::nb2_3_to_i7_msg::ConstPtr& msg) 
{
    /**
     * Copy topic msg data into packet data structure for writing to the port
     */
    /*pkt.counter         	= msg->counter;
    //pkt.clock				= msg->clock;
    pkt.acc_x  				= msg->acc_x/0.00025*65536.0;
    pkt.acc_y  				= msg->acc_y/0.00025*65536.0;
    pkt.acc_z  				= msg->acc_z/0.00025*65536.0;
    pkt.rate_p          	= msg->rate_p/0.02*65536.0;
    pkt.rate_q          	= msg->rate_q/0.02*65536.0;
    pkt.rate_r          	= msg->rate_r/0.02*65536.0;
    pkt.num_imus			= msg->num_imus;
    pkt.pause_switch		= msg->pause_switch;
    pkt.main_loop_counter	= msg->main_loop_counter;*/
}

void NB2_3_To_I7::publishMsg(ros::Publisher* pub_ptr)
{
	pub_ptr->publish(msg);
}


void NB2_3_To_I7::subscribeMsg()
{
	sub = nh.subscribe<messages::nb2_3_to_i7_msg>("messages/nb23out/wrongmsg", 1, &NB2_3_To_I7::packMsg, this);
}


void NB2_3_To_I7::debug_print_packet_data(int res)
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
