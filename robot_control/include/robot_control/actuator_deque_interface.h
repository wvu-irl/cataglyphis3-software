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

#ifndef ACTUATOR_DEQUE_INTERFACE_H
#define ACTUATOR_DEQUE_INTERFACE_H
#include <deque>
#include "task.h"
#include "task_type_enum.h"
#include "drive_halt.h"
#include "drive_straight.h"
#include "drive_pivot.h"
#include "drive_straight_cl.h"
#include "grabber_halt.h"
#include "grabber_set_drop.h"
#include "grabber_set_slides.h"
#include "grabber_idle.h"
#include "vision_halt.h"
#include "vision_search.h"
#define TASK_POOL_SIZE 100
#define PI 3.14159265359

class ActuatorDequeInterface
{
public:
	static std::deque <Task*> driveDeque;
	static std::deque <Task*> grabberDeque;
	static std::deque <Task*> visionDeque;
	static unsigned int taskPoolIndex[NUM_TASKS];
	static Task* taskPool[NUM_TASKS][TASK_POOL_SIZE];
	static DriveHalt driveHalt;
	static GrabberHalt grabberHalt;
	static GrabberIdle grabberIdle;
	static VisionHalt visionHalt;
	static int driveDequeEnded;
	static int driveDequeEmpty;
	static int driveDequeEmptyPrev;
	static int grabberDequeEnded;
	static int grabberDequeEmpty;
	static int grabberDequeEmptyPrev;
	static int visionDequeEnded;
	static int visionDequeEmpty;
	static int visionDequeEmptyPrev;
};

std::deque <Task*> ActuatorDequeInterface::driveDeque;
std::deque <Task*> ActuatorDequeInterface::grabberDeque;
std::deque <Task*> ActuatorDequeInterface::visionDeque;
unsigned int ActuatorDequeInterface::taskPoolIndex[NUM_TASKS] = {0};
Task* ActuatorDequeInterface::taskPool[NUM_TASKS][TASK_POOL_SIZE] = {0};
DriveHalt ActuatorDequeInterface::driveHalt;
GrabberHalt ActuatorDequeInterface::grabberHalt;
GrabberIdle ActuatorDequeInterface::grabberIdle;
VisionHalt ActuatorDequeInterface::visionHalt;
int ActuatorDequeInterface::driveDequeEnded;
int ActuatorDequeInterface::driveDequeEmpty;
int ActuatorDequeInterface::driveDequeEmptyPrev = 1;
int ActuatorDequeInterface::grabberDequeEnded;
int ActuatorDequeInterface::grabberDequeEmpty;
int ActuatorDequeInterface::grabberDequeEmptyPrev = 1;
int ActuatorDequeInterface::visionDequeEnded;
int ActuatorDequeInterface::visionDequeEmpty;
int ActuatorDequeInterface::visionDequeEmptyPrev = 1;
#endif // ACTUATOR_DEQUE_INTERFACE_H
