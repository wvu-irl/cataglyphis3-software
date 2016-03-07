#ifndef HMS_ACTION_SUB_SUPERCLASS_H
#define HMS_ACTION_SUB_SUPERCLASS_H
#include <ros/ros.h>
#include <hsm/HSM_Action.h>
#include <string>

class HSM_Action_Sub
{
public:
	// Members
	ros::NodeHandle nh;
	ros::Subscriber sub;
	// Methods
	virtual void actionCallback(const hsm::HSM_Action::ConstPtr& msg_in) = 0;
};

#endif /* HMS_ACTION_SUB_SUPERCLASS_H */
