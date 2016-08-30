#include <robot_control/action.h>

void Action::pushTask(TASK_TYPE_T taskType)
{
	if(taskType==_driveHalt_ || taskType==_driveStraight_ || taskType==_driveStraightCL_ || taskType==_pivot_)
	{
		ROS_DEBUG("before drive deque push back");
		driveDeque.push_back(taskPool[taskType][taskPoolIndex[taskType]]);
		ROS_DEBUG("after drive deque push back");
		ROS_DEBUG("drive deque size: %u",driveDeque.size());
	}
	else if(taskType==_grabberHalt_ || taskType==_grabberSetSlides_ || taskType==_grabberSetDrop_)
		grabberDeque.push_back(taskPool[taskType][taskPoolIndex[taskType]]);
	else if(taskType==_visionHalt_ || taskType==_search_ || taskType==_approach_ || taskType==_confirmCollect_)
		visionDeque.push_back(taskPool[taskType][taskPoolIndex[taskType]]);
	else ROS_ERROR("attempted to push back invalid TASK type");
	taskPoolIndex[taskType]++;
    if(taskPoolIndex[taskType]>=TASK_POOL_SIZE) taskPoolIndex[taskType] = 0;
}

int Action::runDeques()
{
	driveDequeEmpty = driveDeque.empty();
	if(driveDequeEmptyPrev && !driveDequeEmpty) driveDeque.front()->init();
    else if(!driveDequeEmptyPrev && driveDequeEmpty) driveHalt.init();
    if(driveDequeEmpty) {driveHalt.run(); driveDequeEmpty = 1; driveDequeEnded = 0;}
	else driveDequeEnded = driveDeque.front()->run();
	if(driveDequeEnded!=0)
	{
		driveDeque.pop_front();
		driveDequeEmpty = driveDeque.empty();
		if(driveDequeEmpty==0) driveDeque.front()->init();
	}

	grabberDequeEmpty = grabberDeque.empty();
	if(grabberDequeEmptyPrev && !grabberDequeEmpty) grabberDeque.front()->init();
	if(grabberDequeEmpty) {grabberIdle.run(); grabberDequeEmpty = 1; grabberDequeEnded = 0;}
	else grabberDequeEnded = grabberDeque.front()->run();
	if(grabberDequeEnded!=0)
	{
        if(dropFailed_) grabberDeque.clear();
        else grabberDeque.pop_front();
		grabberDequeEmpty = grabberDeque.empty();
		if(grabberDequeEmpty==0) grabberDeque.front()->init();
	}

	visionDequeEmpty = visionDeque.empty();
	if(visionDequeEmptyPrev && !visionDequeEmpty) visionDeque.front()->init();
	else if(!visionDequeEmptyPrev && visionDequeEmpty) visionHalt.init();
	if(visionDequeEmpty) {visionHalt.run(); visionDequeEmpty = 1; visionDequeEnded = 0;}
	else visionDequeEnded = visionDeque.front()->run();
	if(visionDequeEnded!=0)
	{
		visionDeque.pop_front();
		visionDequeEmpty = visionDeque.empty();
		if(visionDequeEmpty==0) visionDeque.front()->init();
	}

	driveDequeEmptyPrev = driveDequeEmpty;
	grabberDequeEmptyPrev = grabberDequeEmpty;
	visionDequeEmptyPrev = visionDequeEmpty;
	if(driveDequeEmpty && grabberDequeEmpty && visionDequeEmpty) return 1;
	else return 0;
}

void Action::clearDeques()
{
    driveDeque.clear();
    grabberDeque.clear();
    visionDeque.clear();
    //driveDequeEmptyPrev = 1;
    //grabberDequeEmptyPrev = 1;
    //visionDequeEmptyPrev = 1;
}

void Action::initDequesFront()
{
    if(!driveDeque.empty()) driveDeque.front()->init();
    if(!grabberDeque.empty()) grabberDeque.front()->init();
    if(!visionDeque.empty()) visionDeque.front()->init();
}
