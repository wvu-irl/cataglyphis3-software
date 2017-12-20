/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

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
