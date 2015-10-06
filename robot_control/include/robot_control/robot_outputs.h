#ifndef ROBOT_OUTPUTS_H
#define ROBOT_OUTPUTS_H
#include <stdint.h>

class RobotOutputs
{
public:
    // Drive outputs; Range: [-1000,1000] positive is forward, negative is reverse
    int16_t front_left_motor_speed = 0;
    int16_t middle_left_motor_speed = 0;
    int16_t back_left_motor_speed = 0;
    int16_t front_right_motor_speed = 0;
    int16_t middle_right_motor_speed = 0;
    int16_t back_right_motor_speed = 0;
    // Grabber output
    int16_t slide_pos_cmd = 1000;
    int16_t drop_pos_cmd = -1000;
    uint8_t grabber_stop_cmd = 0;
    // Vision output
    //vision_mode_t vision_cmd; // See "vision_states.h"
};

#endif // ROBOT_OUTPUTS_H
