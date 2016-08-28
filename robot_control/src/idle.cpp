#include <robot_control/idle.h>

void Idle::init()
{
    clearDeques();
    driveHalt.init();
    visionHalt.init();
    dropFailed_ = false;
    slidesFailed_ = false;
}

int Idle::run()
{
    driveHalt.run();
    grabberHalt.run();
    visionHalt.run();
    if(driveDeque.empty()) driveDequeEmptyPrev = 1;
    if(grabberDeque.empty()) grabberDequeEmptyPrev = 1;
    if(visionDeque.empty()) visionDequeEmptyPrev = 1;
}
