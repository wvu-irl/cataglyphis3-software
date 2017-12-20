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

#include "HSM_NB1_Monitor_class.h"

HSM_NB1_Monitor_class::HSM_NB1_Monitor_class() //constructor
{
	ros::NodeHandle nh;
	//Publisher for reset signal for NB1 over DIO line
	pub_action = nh.advertise<hsm::HSM_Action>("HSM_Act/NB1_reset",1000);

	//Subscribe to NB1 UDP HSM detection
	sub_udp_detection = nh.subscribe<hsm::HSM_Detection>("HSM_Det/NB1_udp_receiver",1,&HSM_NB1_Monitor_class::detectionCallback_udp, this);

	//Subscribe to NB1 Serial HSM detection
	sub_serial_detection = nh.subscribe<hsm::HSM_Detection>("HSM_Det/NB1_serial_receiver",1,&HSM_NB1_Monitor_class::detectionCallback_serial, this);

	//Recovering mode timer
	timer = nh.createTimer(ros::Duration(10),&HSM_NB1_Monitor_class::recoveringTimerCallback,this,true);
	timer.stop();

	bad_read_detection_threshold = 4;      //per I/F spec should be 20
	simultaneous_bad_reads_threshold = 3;  //per I/F spec should be 15
	timeout_detection_threshold = 2;       //per I/F spec should be 10
	node_restart_threshold = 20;
	init_count = 0;

	udp_missing_packet = false;
	serial_missing_packet = false;
	udp_bad_reads_flag = false;
	serial_bad_reads_flag = false;

	udp_detection_flag = false;
	serial_detection_flag = false;
	monitor_state = Init_Monitor;
	recovering_substate = Commanding;

}

void HSM_NB1_Monitor_class::service_monitor()
{
	// Debug outputs
	//std::cout << "monitor_state = " << monitor_state << std::endl;

	// Monitor state machine
	switch(monitor_state)
	{
		case Init_Monitor:
			usleep(1000000); // Sleep 1 second to let publisher fully initialize before first publish
			msg_out.command = "UDP PUBLISH";
			pub_action.publish(msg_out);
			on_udp = true;
			//std::cout << "Init - UDP PUBLISH" << std::endl;

			init_count++;
			if(init_count>=3){monitor_state = Monitoring;}
			else monitor_state = Init_Monitor;

			break;
		case Monitoring:
			if(udp_detection_flag)
			{
				// Debug outputs
				//std::cout << "message_type = " << udp_detection_data.message_type << std::endl;
				//std::cout << "count = " << udp_detection_data.count << std::endl;

				udp_detection_flag = false;
				if((udp_detection_data.message_type == "BAD PACKET") ||
				   (udp_detection_data.message_type == "PORT_READ ERROR") ||
				   (udp_detection_data.message_type == "TIMEOUT"))
				{
					udp_bad_read_detection_count++;
					if((udp_bad_read_detection_count()>=bad_read_detection_threshold) &&
					   ((on_udp) || (udp_node_restart_count()<node_restart_threshold) ||
					    (serial_bad_read_detection_count()+serial_timeout_detection_count()>0)))
					{
						recovering_error_type = "UDP_BAD_READ";
						udp_bad_read_detection_count.reset();
						monitor_state = Recovering;
					}
					else if(udp_bad_read_detection_count()>=simultaneous_bad_reads_threshold)
					{
						if(serial_bad_reads_flag) //trigger NB1 reset if both serial and udp have bad reads
						{
							//reset all counters and flags
							udp_timeout_detection_count.reset();
							udp_bad_read_detection_count.reset();
							serial_timeout_detection_count.reset();
							serial_bad_read_detection_count.reset();
							udp_missing_packet = false;
							serial_missing_packet = false;
							udp_bad_reads_flag = false;
							serial_bad_reads_flag = false;
							//transition
							recovering_error_type = "NB1_RESET";
							monitor_state = Recovering;
						}
						else  //if not serial yet, set udp flag and wait for serial to trip or good packet
						{
							udp_bad_reads_flag = true;
							//std::cout << "udp_bad_reads_flag=true" << std::endl;
						}
					}
					if(udp_detection_data.message_type == "TIMEOUT")
					{
						udp_timeout_detection_count++;
						if(udp_timeout_detection_count()>=timeout_detection_threshold)
						{
							if(serial_missing_packet) //trigger NB1 reset if both serial and udp have missing packets
							{
								//reset all counters and flags
								udp_timeout_detection_count.reset();
								udp_bad_read_detection_count.reset();
								serial_timeout_detection_count.reset();
								serial_bad_read_detection_count.reset();
								udp_missing_packet = false;
								serial_missing_packet = false;
								udp_bad_reads_flag = false;
								serial_bad_reads_flag = false;
								//transition
								recovering_error_type = "NB1_RESET";
								monitor_state = Recovering;
							}
							else {udp_missing_packet = true;}
						}
						else monitor_state = Monitoring;
					}
				}
				else if(udp_detection_data.message_type == "GOOD")
				{
					udp_bad_read_detection_count.reset();
					udp_timeout_detection_count.reset();
					udp_bad_reads_flag = false;
					udp_missing_packet = false;
					monitor_state = Monitoring;
				}
			}
			else if(serial_detection_flag)
			{
				// Debug outputs
				//std::cout << "message_type = " << serial_detection_data.message_type << std::endl;
				//std::cout << "count = " << serial_detection_data.count << std::endl;

				serial_detection_flag = false;
				if((serial_detection_data.message_type == "BAD PACKET") ||
				   (serial_detection_data.message_type == "PORT_READ ERROR") ||
				   (serial_detection_data.message_type == "TIMEOUT"))
				{
					serial_bad_read_detection_count++;
					if((serial_bad_read_detection_count()>=bad_read_detection_threshold) &&
					   (!(on_udp) ||(serial_node_restart_count()<node_restart_threshold) ||
					    (udp_bad_read_detection_count()+udp_timeout_detection_count()>0)))

					{
						recovering_error_type = "SERIAL_BAD_READ";
						serial_bad_read_detection_count.reset();
						monitor_state = Recovering;
					}
					else if(serial_bad_read_detection_count()>=simultaneous_bad_reads_threshold)
					{
						if(udp_bad_reads_flag) //trigger NB1 reset if both serial and udp have bad reads
						{
							//reset all counters and flags
							udp_timeout_detection_count.reset();
							udp_bad_read_detection_count.reset();
							serial_timeout_detection_count.reset();
							serial_bad_read_detection_count.reset();
							udp_missing_packet = false;
							serial_missing_packet = false;
							udp_bad_reads_flag = false;
							serial_bad_reads_flag = false;
							//transition
							recovering_error_type = "NB1_RESET";
							monitor_state = Recovering;
						}
						else  //if not udp yet, set set serial flag and wait for serial to trip or good packet
						{
							serial_bad_reads_flag = true;
							//std::cout << "serial_bad_reads_flag=true"<< std::endl;
						}
					}
					if(serial_detection_data.message_type == "TIMEOUT")
					{
						serial_timeout_detection_count++;
						if(serial_timeout_detection_count()>=timeout_detection_threshold)
						{
							if(udp_missing_packet) //trigger NB1 reset if both serial and udp have missing packets
							{
								//reset all counters and flags
								udp_timeout_detection_count.reset();
								udp_bad_read_detection_count.reset();
								serial_timeout_detection_count.reset();
								serial_bad_read_detection_count.reset();
								udp_missing_packet = false;
								serial_missing_packet = false;
								udp_bad_reads_flag = false;
								serial_bad_reads_flag = false;
								//transition
								recovering_error_type = "NB1_RESET";
								monitor_state = Recovering;
							}
							else {serial_missing_packet = true;}
						}
						else monitor_state = Monitoring;
					}
				}
				else if(serial_detection_data.message_type == "GOOD")
				{
					serial_bad_read_detection_count.reset();
					serial_timeout_detection_count.reset();
					serial_missing_packet = false;
					serial_bad_reads_flag = false;
					monitor_state = Monitoring;
				}
			}
			break;
		case Recovering:
			if(recovering_error_type=="UDP_BAD_READ")
			{
				switch(recovering_substate)
				{
					case Commanding:
						msg_out.command = "SERIAL PUBLISH";
						pub_action.publish(msg_out);
						on_udp = false;
						udp_node_restart_count++;
						monitor_state = Recovering;
						recovering_substate = Confirming;
						break;
					case Confirming: // For this test, there is no confirmation, only start timer
						timer.setPeriod(ros::Duration(10));
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
			else if(recovering_error_type=="SERIAL_BAD_READ")
			{
				switch(recovering_substate)
				{
					case Commanding:
						msg_out.command = "UDP PUBLISH";
						pub_action.publish(msg_out);
						on_udp = true;
						serial_node_restart_count++;
						monitor_state = Recovering;
						recovering_substate = Confirming;
						break;
					case Confirming: // For this test, there is no confirmation, only start timer
						timer.setPeriod(ros::Duration(10));
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
			else if(recovering_error_type=="NB1_RESET")
			{
				switch(recovering_substate)
				{
					case Commanding:
						msg_out.command = "NB1 RESET";
						pub_action.publish(msg_out);
						udp_node_restart_count.reset();
						serial_node_restart_count.reset();
						monitor_state = Recovering;
						recovering_substate = Confirming;
						break;
					case Confirming: // For this test, there is no confirmation, only start timer
						timer.setPeriod(ros::Duration(10));
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
}

void HSM_NB1_Monitor_class::detectionCallback_udp(const hsm::HSM_Detection::ConstPtr& msg_in)
{
	udp_detection_data.detector_node = msg_in->detector_node;
	udp_detection_data.message_type = msg_in->message_type;
	udp_detection_data.count = msg_in->count;
	udp_detection_flag = true;
}

void HSM_NB1_Monitor_class::detectionCallback_serial(const hsm::HSM_Detection::ConstPtr& msg_in)
{
	serial_detection_data.detector_node = msg_in->detector_node;
	serial_detection_data.message_type = msg_in->message_type;
	serial_detection_data.count = msg_in->count;
	serial_detection_flag = true;
}

void HSM_NB1_Monitor_class::recoveringTimerCallback(const ros::TimerEvent& event)
{
	monitor_state = Monitoring;
	recovering_substate = Commanding;
}
