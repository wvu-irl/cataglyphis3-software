#include "HSM_Heartbeat_Monitor_class.h"

//constructor
HSM_Heartbeat_Monitor_class::HSM_Heartbeat_Monitor_class(std::string node_name, int node_Hz) 
{
	//std::cout << "HSM_Heartbeat_Monitor constructor start" << std::endl;
	//std::cout << "HB node = " << node_name << std::endl;
	//std::cout << "HB node freq = " << node_Hz << std::endl;

	//initialize node monitor variables
	hb_node_name = node_name;
	hb_node_Hz = node_Hz;
	hb_node_died = false;
	hb_timeout_period = ros::Duration(1000/(node_Hz)); // timeout = 10X node execution period
		
	ros::NodeHandle nh;
	//Publisher for node restart signal
	pub_action = nh.advertise<hsm::HSM_Action>("HSM_Act/node_restart",1000);
	msg_out.command = node_name;
	
	//Subscribe to heartbeat detection
	sub_heartbeats = nh.subscribe<hsm::HSM_Detection>("HSM_Det/"+hb_node_name+"/HB",1,&HSM_Heartbeat_Monitor_class::detectionCallback_heartbeat, this); 
	
	//heartbeat timers initiated with a start up timeout
	heartbeat_timer = nh.createTimer(ros::Duration(NODE_STARTUP_TIMEOUT), 
							&HSM_Heartbeat_Monitor_class::heartbeatTimerCallback,this,true); 
	heartbeat_timer.stop();
	
	//monitor state machine initialization
	monitor_state = Init_Monitor;
	recovering_substate = Commanding;

	//Recovering mode timer
	recovery_timer = nh.createTimer(ros::Duration(10),&HSM_Heartbeat_Monitor_class::recoveringTimerCallback,this,true); 
	recovery_timer.stop();

	//std::cout << "" << std::endl;
	//std::cout << "HB monitor constructed" << std::endl;
}


void HSM_Heartbeat_Monitor_class::service_monitor()
{
	// Debug outputs
	//std::cout << "monitor_state = " << monitor_state << std::endl;

	// Monitor state machine
	switch(monitor_state)
	{
		case Init_Monitor:
			//std::cout << "" << std::endl;
			//std::cout << "Init_Monitor" << std::endl;
			usleep(30000); // Sleep .03 second during startup
			//start all heartbeat timers
			heartbeat_timer.start();		
			monitor_state = Monitoring;
			break;
		
		case Monitoring:
			// Debug outputs
			//std::cout << "" << std::endl;
			//std::cout << "Monitoring: count = " << heartbeat_data.count << std::endl;
			if(hb_node_died)
			{
				//std::cout << "node died" << std::endl << std::endl << std::endl << std::endl;
				monitor_state = Recovering;
				recovering_substate = Commanding;
				hb_node_died = false;
				monitor_trigger_count++;
			}
			else monitor_trigger_count = 0;
			break;
		case Recovering:
			switch(recovering_substate)
			{
				case Commanding:
					//std::cout << "" << std::endl << "Recovering: Commanding" << std::endl;
//						msg_out.command = heartbeat_data.detector_node;
					//pub_action.publish(msg_out);
					/*if(monitor_trigger_count > monitor_trigger_limit) */ //system_string = "kill $(ps aux | grep /"+hb_node_name+" | grep -v grep | awk '{print$2}')";
					/*else */system_string = "rosnode kill /" + hb_node_name;
					system(system_string.c_str());
					monitor_state = Recovering;
					recovering_substate = Confirming;
					break;
				case Confirming: // For this test, there is no confirmation, only start timer, using heartbeat timer in startup mode
					//std::cout << "" << std::endl << "Recovering: Confirming" << std::endl;
					heartbeat_timer.setPeriod(ros::Duration(NODE_STARTUP_TIMEOUT));
					heartbeat_timer.start();
					monitor_state = Recovering;
					recovering_substate = Waiting;
					//std::cout << "" << std::endl << "Recovering: Waiting" << std::endl;
					break;
				case Waiting:
					monitor_state = Recovering;
					recovering_substate = Waiting;
					break;
			}
			break;
	} //switch
} //method

void HSM_Heartbeat_Monitor_class::detectionCallback_heartbeat(const hsm::HSM_Detection::ConstPtr& msg_in)
{
	heartbeat_data.detector_node = msg_in->detector_node;
	heartbeat_data.message_type = msg_in->message_type;
	heartbeat_data.count = msg_in->count;
	
	//need to feed the corresponding heartbeat timer
	//int i = nodeLookup(heartbeat_data.detector_node);
	heartbeat_timer.stop();  //~~~may not need to stop or setPeriod to restart
	heartbeat_timer.setPeriod(hb_timeout_period);
	heartbeat_timer.start();
	
	//~~~decide whether to add transition from bad to recovered here or startup tracking
	if(recovering_substate==Waiting) {monitor_state = Monitoring;} //avoid catching a late heartbeat just as commanding node restart
	
}

//need to duplicate per max number of nodes
void HSM_Heartbeat_Monitor_class::recoveringTimerCallback(const ros::TimerEvent& event)
{
	monitor_state = Monitoring;
	recovering_substate = Commanding;
}

//need to duplicate per max number of nodes
void HSM_Heartbeat_Monitor_class::heartbeatTimerCallback(const ros::TimerEvent& event)
{
	//needs to set failed flag for the failed node
	hb_node_died = true;
	monitor_state = Monitoring;
	recovering_substate = Commanding;
}
