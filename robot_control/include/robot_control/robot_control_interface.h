#ifndef ROBOT_CONTROL_INTERFACE_H
#define ROBOT_CONTROL_INTERFACE_H
#include "robot_outputs.h"
#include "robot_status.h"
#include <messages/CVSearchCmd.h>
#include <ros/ros.h>

class RobotControlInterface
{
public:
    static RobotStatus robotStatus;
    static RobotOutputs robotOutputs;
	static ros::ServiceClient cvSearchCmdClient;
	static messages::CVSearchCmd cvSearchCmdSrv;
};

RobotStatus RobotControlInterface::robotStatus;
RobotOutputs RobotControlInterface::robotOutputs;
ros::ServiceClient RobotControlInterface::cvSearchCmdClient;
messages::CVSearchCmd RobotControlInterface::cvSearchCmdSrv;

#endif // ROBOT_CONTROL_INTERFACE_H
