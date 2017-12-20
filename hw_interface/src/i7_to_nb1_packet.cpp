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

/**
 * i7_to_nb1_packet.cpp
 * Implementation of methods for the WVU-NSRR Packet Structured Serial #2 Class
 * for stuffing topic msg and debug print
 */
#include "i7_to_nb1_packet.h"

I7_To_NB1::I7_To_NB1(buffer_RW_t buffer_RW)
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
	H3_def = '2';
	pkt.H1 = H1_def;
	pkt.H2 = H2_def;
	pkt.H3 = H3_def;
}

void I7_To_NB1::unpackMsg()
{
    /**
     * Copy packet data into topic msg data structure for ROS publishing
     */
    msg.x_position				= pkt.x_position;
    msg.y_position				= pkt.y_position;
    msg.heading					= pkt.heading;
    msg.north_angle				= pkt.north_angle;
    msg.waypoint_index			= pkt.waypoint_index;
    msg.waypoint_list_type		= pkt.waypoint_list_type;
    msg.samples_collected		= pkt.samples_collected;
    msg.init_complete			= pkt.init_complete;
    msg.using_serial			= pkt.using_serial;
}

void I7_To_NB1::packMsg(const messages::i7_to_nb1_msg::ConstPtr& msg)
{
    /**
     * Copy topic msg data into packet data structure for writing to the port
     */
    pkt.counter         		= incrementCounter();
    pkt.x_position				= msg->x_position;
    pkt.y_position				= msg->y_position;
    pkt.heading					= msg->heading;
    pkt.north_angle				= msg->north_angle;
    pkt.waypoint_index			= msg->waypoint_index;
    pkt.waypoint_list_type		= msg->waypoint_list_type;
    pkt.samples_collected		= msg->samples_collected;
    pkt.init_complete			= msg->init_complete;
    pkt.using_serial			= msg->using_serial;
    pkt.checksum                = computeChecksum(buf, wbuf_size);
}

void I7_To_NB1::publishMsg(ros::Publisher* pub_ptr)
{
	pub_ptr->publish(msg);
}


void I7_To_NB1::subscribeMsg()
{
	sub = nh.subscribe<messages::i7_to_nb1_msg>("messages/nb1out/nb1out", 1, &I7_To_NB1::packMsg, this);
}


void I7_To_NB1::debug_print_packet_data(int res)
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
