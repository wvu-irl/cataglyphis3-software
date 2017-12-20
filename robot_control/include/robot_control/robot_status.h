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

#ifndef ROBOT_STATUS_H
#define ROBOT_STATUS_H

class RobotStatus
{
public:
	unsigned int loopRate = 20; // Hz. Default to 20 Hz
	// Drive data
	float xPos = 0.0; // m
	float yPos = 0.0; // m
	float heading = 0.0; // deg
	float bearing = 0.0; // deg
	bool homingUpdated = false;
	float distToOrigin = 0.0; // m
	float yawRate = 0.0; // deg/s
	float rollAngle = 0.0; // deg
	float pitchAngle = 0.0; // deg
	float velocity = 0.0; // m/s
	float vMax = 1.2; // m/s
	float rMax = 45.0; // deg/s
	long int flEncoder = 0;
	long int mlEncoder = 0;
	long int blEncoder = 0;
	long int frEncoder = 0;
	long int mrEncoder = 0;
	long int brEncoder = 0;
	// Grabber data
	int grabberSlideStatus = 0;
	int grabberDropStatus = 0;
	int grabberSlidePos = 0;
	int grabberDropPos = 0;
	// Vision data
	float panAngle = 0.0; // deg
	float neckPos = 0.0; // deg
        // NetBurner data
    bool pauseSwitch = true;
};

#endif // ROBOT_STATUS_H
