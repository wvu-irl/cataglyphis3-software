#ifndef ROBOT_CONTROL_INTERFACE_H
#define ROBOT_CONTROL_INTERFACE_H
#include "robot_outputs.h"
#include "robot_status.h"
#include <ros/ros.h>

class RobotControlInterface
{
public:
    static RobotStatus robotStatus;
    static RobotOutputs robotOutputs;
};

RobotStatus RobotControlInterface::robotStatus;
RobotOutputs RobotControlInterface::robotOutputs;

#endif // ROBOT_CONTROL_INTERFACE_H
