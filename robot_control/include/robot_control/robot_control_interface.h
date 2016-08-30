#ifndef ROBOT_CONTROL_INTERFACE_H
#define ROBOT_CONTROL_INTERFACE_H
#include "robot_outputs.h"
#include "robot_status.h"
#include <messages/CVSearchCmd.h>
#include <ros/ros.h>
#include "bit_utils.h"
#define GRABBER_OPEN 1000
#define GRABBER_CLOSED -900
#define GRABBER_DROPPED 1000
#define GRABBER_RAISED -1000

class RobotControlInterface
{
public:
    static RobotStatus robotStatus;
    static RobotOutputs robotOutputs;
	static ros::ServiceClient cvSearchCmdClient;
	static messages::CVSearchCmd cvSearchCmdSrv;
	static Leading_Edge_Latch dropStatusLEL_;
	static Leading_Edge_Latch slideStatusLEL_;
	static bool dropEnded_;
	static bool slidesEnded_;
	static bool dropFailed_;
	static bool slidesFailed_;
	const int dropTol_ = 150;
	const int slideTol_ = 80;
};

RobotStatus RobotControlInterface::robotStatus;
RobotOutputs RobotControlInterface::robotOutputs;
ros::ServiceClient RobotControlInterface::cvSearchCmdClient;
messages::CVSearchCmd RobotControlInterface::cvSearchCmdSrv;
Leading_Edge_Latch RobotControlInterface::dropStatusLEL_;
Leading_Edge_Latch RobotControlInterface::slideStatusLEL_;
bool RobotControlInterface::dropEnded_;
bool RobotControlInterface::slidesEnded_;
bool RobotControlInterface::dropFailed_;
bool RobotControlInterface::slidesFailed_;

#endif // ROBOT_CONTROL_INTERFACE_H
