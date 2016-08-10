#ifndef BIAS_REMOVAL_H
#define BIAS_REMOVAL_H
#include "procedure.h"

class BiasRemoval : public Procedure
{
public:
	// Members
	ros::NodeHandle nh;
	const float biasRemovalActionTimeoutTime = 20.0; // sec
	bool biasRemovalTimedOut;
	// Methods
	bool runProc();
	BiasRemoval();
	void callback(const ros::TimerEvent& event);
};

#endif // BIAS_REMOVAL_H
