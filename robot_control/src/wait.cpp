#include <robot_control/wait.h>

void Wait::init()
{
	waitTime_ = params.float1;
	timeExpired_ = false;
	timer.stop();
	timer.setPeriod(ros::Duration(waitTime_));
	clearDeques();
	driveHalt.init();
	visionHalt.init();
	timer.start();
}

int Wait::run()
{
	driveHalt.run();
	grabberIdle.run();
	visionHalt.run();
	if(driveDeque.empty()) driveDequeEmptyPrev = 1;
	if(grabberDeque.empty()) grabberDequeEmptyPrev = 1;
	if(visionDeque.empty()) visionDequeEmptyPrev = 1;
	return timeExpired_;
}

void Wait::waitTimeCallback_(const ros::TimerEvent &event)
{
	timeExpired_ = true;
	timer.stop();
}
