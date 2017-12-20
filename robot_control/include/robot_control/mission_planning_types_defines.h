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

#ifndef MISSION_PLANNING_TYPES_DEFINES_H
#define MISSION_PLANNING_TYPES_DEFINES_H

#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI
#define NUM_PROC_TYPES 17
#define MAX_SAMPLES 10
#define NUM_TIMERS 7
#define NUM_SAMPLE_TYPES 7 // !!! Change switch case in findHighestConfSample() if this number is ever changed
// !!! If PROC_TYPES_T is ever edited, edit controlCallback_ in MissionPlanning as well
enum PROC_TYPES_T {__initialize__, __emergencyEscape__,__avoid__, __biasRemoval__, __nextBestRegion__, __searchRegion__, __examine__, __approach__, __collect__, __confirmCollect__, __reorient__, __goHome__, __squareUpdate__, __depositApproach__, __depositSample__, __safeMode__, __sosMode__};
enum TIMER_NAMES_T {_roiTimer_, _biasRemovalTimer_, _homingTimer_, _biasRemovalActionTimeoutTimer_, _searchTimer_, _queueEmptyTimer_, _roiOvertimeTimer_};
enum PROC_STATE_T {_init_, _exec_, _interrupt_, _finish_};
struct CV_SAMPLE_PROPS_T
{
	float confidence = 0.0;
	bool types[NUM_SAMPLE_TYPES] = {false};
	float distance = 0.0;
	float bearing = 0.0;
};

#endif //MISSION_PLANNING_TYPES_DEFINES_H
