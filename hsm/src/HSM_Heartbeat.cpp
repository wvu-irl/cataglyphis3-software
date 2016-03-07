#include "HSM_Heartbeat.h"

HSM_Heartbeat::HSM_Heartbeat() // Constructor
{
	postfix = "HB";
	beat_detector = new HSM_Detector(postfix);
	//std::cout << "HSM_Heartbeat initialized" << std::endl;
}


void HSM_Heartbeat::Beat()
{
	beat_count++;
	beat_detector->msg.count = beat_count();
	beat_detector->HSM_notify("HEARTBEAT");
}

