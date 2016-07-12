#include "Serial_HSM_Act_class.h"


Serial_HSM_Act::Serial_HSM_Act(bool* publish_enable_ptr_input)
{
/*
	sub = nh.subscribe<hsm::HSM_Action>("HSM_Act/port_pub_enabled", 1000, &Serial_HSM_Act::actionCallback, this);
	publish_enable_ptr = publish_enable_ptr_input;
	*/
	
	*publish_enable_ptr = false;
}

void Serial_HSM_Act::actionCallback(const hsm::HSM_Action::ConstPtr& msg_in)
{
/*
	if(msg_in->command=="SERIAL PUBLISH") *publish_enable_ptr = true;
	else if(msg_in->command=="UDP PUBLISH") *publish_enable_ptr = false;
	*/
}
