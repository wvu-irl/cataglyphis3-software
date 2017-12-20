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

#ifndef ROBOT_SIM_H
#define ROBOT_SIM_H
#include <math.h>
#include <stdint.h>

#define RAD2DEG 180.0/3.14159
#define DEG2RAD 3.14159/180.0

class RobotSim
{
public:
	// Members
	// Drive
	double xPos; // m
	double yPos; // m
	double heading; // deg
	// Grabber
	int slidePos;
	int slidePosCmdPrev;
	int dropPos;
	int dropPosCmdPrev;
	int slideStop;
	int dropStop;
	// Netburner
	uint8_t nb1PauseSwitch = 255;
	// Sim
	const double normalSpeedDT = 0.05;  // Default of 20 Hz, resulting in 1x speed
	double dt = normalSpeedDT;
	// Methods
	RobotSim(double initX, double initY, double initHeading, double simRate); // Constructor
	void drive(double linV, double angV); // Drive robot using linear and angular velocities as input. Arg units: m/s, deg/s
	void teleport(double teleX, double teleY, double teleHeading); // Teleport robot to location. Arg units: m, m, deg
	void runGrabber(int slidePosCmd, int dropPosCmd, int slideStopCmd, int dropStopCmd); // Manage grabber operations
private:
	const double slideSpeed_ = 2000.0/5.0; // full range of -1000 to 1000 in 5 seconds
	const double dropSpeed_ = 2000.0/5.0; // full range of -1000 to 1000 in 5 seconds
};

#endif // ROBOT_SIM_H
