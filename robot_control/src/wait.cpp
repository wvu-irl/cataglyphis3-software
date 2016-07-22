#include <robot_control/wait.h>

Wait::Wait()
{
	waitTimer = nh.createTimer(ros::Duration(10.0), &Wait::waitTimeCallback_, this);
	waitTimer.stop();
	waitTimerActive = false;
}

void Wait::init()
{
	waitTime_ = params.float1;
	timeExpired_ = false;
	waitTimer.stop();
	waitTimer.setPeriod(ros::Duration(waitTime_));
	clearDeques();
	driveHalt.init();
	visionHalt.init();
	waitTimer.start();
	waitTimerActive = true;
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
	waitTimer.stop();
	waitTimerActive = false;
}
