#ifndef HSM_HEARTBEAT_H
#define HSM_HEARTBEAT_H

#include "HSM_Detector.h"
#include "Counter.h"
#include <string>

class HSM_Heartbeat
{
private:
	// Members
	std::string postfix;
	HSM_Detector* beat_detector;
	Counter beat_count;
	// ROS

	// Methods

public:
	// Members

	// ROS

	// Methods
	HSM_Heartbeat(); // Constructor
//	~HSM_Heartbeat(); // Destructor
	void Beat();
};

#endif /* HSM_HEARTBEAT_H */
