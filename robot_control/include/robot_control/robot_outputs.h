#ifndef ROBOT_OUTPUTS_H
#define ROBOT_OUTPUTS_H
#include <stdint.h>
#include "vision_task_type_enum.h"

class RobotOutputs
{
public:
    // Drive outputs; Range: [-1000,1000] positive is forward, negative is reverse
	int16_t flMotorSpeed = 0;
	int16_t mlMotorSpeed = 0;
	int16_t blMotorSpeed = 0;
	int16_t frMotorSpeed = 0;
	int16_t mrMotorSpeed = 0;
	int16_t brMotorSpeed = 0;
	bool stopFlag = true;
	bool turnFlag = false;
    // Grabber output
	int16_t slidePosCmd = 1000;
	int16_t dropPosCmd = -1000;
	uint8_t grabberStopCmd = 0;
    // Vision output
	VISION_TASK_TYPE_T visionCmd = hold__;
};

#endif // ROBOT_OUTPUTS_H
