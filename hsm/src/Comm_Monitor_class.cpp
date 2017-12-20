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

#include "Comm_Monitor_class.h"

Comm_Monitor_class::Comm_Monitor_class() //constructor
{
	detection_flag = false;
	monitor_state = Init_Monitor;
	recovering_substate = Commanding;

	ros::NodeHandle nh;
	pub = nh.advertise<hsm::HSM_Action>("HSM_Act/port_pub_enabled",1000);
	serial_hsm_sub = nh.subscribe<hsm::HSM_Detection>("HSM_Det/i7_nb1_serial",1,&Comm_Monitor_class::detectionCallback, this);
	udp_hsm_sub = nh.subscribe<hsm::HSM_Detection>("HSM_Det/nb1_i7_udp_receiver",1,&Comm_Monitor_class::detectionCallback, this);
	serial_data_sub = nh.subscribe<messages::nb1_to_i7_msg>("/hw_interface/nb1in/nb1in",1,&Comm_Monitor_class::serialDataCallback,this);
	udp_data_sub = nh.subscribe<messages::nb1_to_i7_msg>("/hw_interface/nb1in/nb1in",1,&Comm_Monitor_class::udpDataCallback,this);
	nb2_data_sub = nh.subscribe<messages::nb2_3_to_i7_msg>("/hw_interface/nb2in/nb2in",1,&Comm_Monitor_class::nb2DataCallback,this);
	nb3_data_sub = nh.subscribe<messages::nb2_3_to_i7_msg>("/hw_interface/nb3in/nb3in",1,&Comm_Monitor_class::nb3DataCallback,this);
	timer = nh.createTimer(ros::Duration(10),&Comm_Monitor_class::recoveringTimerCallback,this,true); //~~~should be an HSM Timer; callback needs object qualification
	timer.stop();
//	timer = new Timer(ros::Duration(10),&Comm_Monitor_class::recoveringTimerCallback,this,true);
	detection_threshold = 1;
	init_count = 0;
	udp_callback_time = ros::Time::now().toSec();
	serial_callback_time = ros::Time::now().toSec();
	current_time = ros::Time::now().toSec();
}

void Comm_Monitor_class::service_monitor()
{
	// Debug outputs
	//std::cout << "monitor_state = " << monitor_state << std::endl;
	//std::cout << "detector_node = " << detection_data.detector_node << std::endl;
	//std::cout << "message_type = " << detection_data.message_type << std::endl;
	//std::cout << "count = " << detection_data.count << std::endl;
//	std::cout << "consecutive_count = " << detection_data.consecutive_count << std::endl;

	// Monitor state machine
	ROS_DEBUG("comm monitor state = %i",monitor_state);
	switch(monitor_state)
	{
		case Init_Monitor:
			usleep(1000000); // Sleep 1 second to let publisher fully initialize before first publish
			msg_out.command = "UDP PUBLISH";
			pub.publish(msg_out);
			//std::cout << "Init - UDP PUBLISH " << std::endl;

			init_count++;
			if(init_count>=3){monitor_state = Monitoring;}
			else monitor_state = Init_Monitor;

			break;
		case Monitoring:
//			timer.stop(); // now stopped in constructor
			current_time = ros::Time::now().toSec();
			if((current_time - udp_callback_time >= callback_timeout_time)&&(msg_out.command == "UDP PUBLISH"))
			{
				detection_flag = true;
				detection_data.detector_node = "/nb1_i7_udp_receiver"; // *** This is going to change
				detection_data.message_type = "NO CONTACT";
				nb1_udp_go = false;
			}
			if(current_time - nb2_callback_time >= callback_timeout_time)
			{
				nb2_go = false;
			}
			if(current_time - nb3_callback_time >= callback_timeout_time)
			{
				nb3_go = false;
			}
			if((current_time - serial_callback_time >= callback_timeout_time)&&msg_out.command == "SERIAL PUBLISH")
			{
				detection_flag = true;
				detection_data.detector_node = "/i7_nb1_serial"; // *** This is going to change
				detection_data.message_type = "NO CONTACT";
				nb1_serial_go = false;
			}
			if(detection_flag)
			{
				if((detection_data.message_type == "BAD PACKET") || (detection_data.message_type == "PORT_READ ERROR") || (detection_data.message_type == "NO CONTACT"))
				{
					if(detection_data.count==0) detection_count++;
					else detection_count.reset();
				}
				if(detection_count()>=detection_threshold)
				{
					recovering_error_type = detection_data.message_type;
					monitor_state = Recovering;
					detection_count.reset();
				}
				else monitor_state = Monitoring;
			}
			else monitor_state = Monitoring;
			detection_flag = false;
			break;
		case Recovering:
			if(recovering_error_type=="BAD PACKET" || recovering_error_type=="PORT_READ ERROR" || recovering_error_type=="NO CONTACT")
			{
				switch(recovering_substate)
				{
					case Commanding:
						if(detection_data.detector_node=="/nb1_i7_udp_receiver") msg_out.command = "SERIAL PUBLISH"; // *** going to change
						else if(detection_data.detector_node=="/i7_nb1_serial") msg_out.command = "UDP PUBLISH"; // *** going to change
						pub.publish(msg_out);
						monitor_state = Recovering;
						recovering_substate = Confirming;
						break;
					case Confirming: // For this test, there is no confirmation, only start timer
						timer.setPeriod(ros::Duration(1));
						timer.start();
						monitor_state = Recovering;
						recovering_substate = Waiting;
						break;
					case Waiting:
						monitor_state = Recovering;
						recovering_substate = Waiting;
						break;
				}
			}
			break;
	}
	nb1_go = nb1_udp_go || nb1_serial_go;
}

void Comm_Monitor_class::detectionCallback(const hsm::HSM_Detection::ConstPtr& msg_in)
{
	detection_data.detector_node = msg_in->detector_node;
	detection_data.message_type = msg_in->message_type;
	detection_data.count = msg_in->count;
	detection_flag = true;
}

void Comm_Monitor_class::recoveringTimerCallback(const ros::TimerEvent& event)
{
	monitor_state = Monitoring;
	recovering_substate = Commanding;
}

void Comm_Monitor_class::udpDataCallback(const messages::nb1_to_i7_msg::ConstPtr& msg_in)
{
	udp_callback_time = ros::Time::now().toSec();
	nb1_udp_go = true;
}

void Comm_Monitor_class::serialDataCallback(const messages::nb1_to_i7_msg::ConstPtr& msg_in)
{
	serial_callback_time = ros::Time::now().toSec();
	nb1_serial_go = true;
}

void Comm_Monitor_class::nb2DataCallback(const messages::nb2_3_to_i7_msg::ConstPtr& msg_in)
{
	nb2_callback_time = ros::Time::now().toSec();
	nb2_go = true;
}

void Comm_Monitor_class::nb3DataCallback(const messages::nb2_3_to_i7_msg::ConstPtr& msg_in)
{
	nb3_callback_time = ros::Time::now().toSec();
	nb3_go = true;
}
