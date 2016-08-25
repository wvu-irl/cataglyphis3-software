#ifndef TASK_H
#define TASK_H
#include "action_params.h"
#include "robot_control_interface.h"
#include <math.h>

class Task : public RobotControlInterface
{
public:
    virtual void init() = 0;
    virtual int run() = 0;
    ACTION_PARAMS_T params;
};

#endif // TASK_H
