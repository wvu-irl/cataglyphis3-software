#include <robot_control/idle.h>

void Idle::init()
{
    driveDeque_.clear();
    grabberDeque_.clear();
    visionDeque_.clear();

    driveDeque_.push_back(); // Need to define action pools somewhere
}
