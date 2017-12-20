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

#include <simulation/RobotSim.h>

RobotSim::RobotSim(double initX, double initY, double initHeading, double simRate)
{
	teleport(initX, initY, initHeading);
    dt = 1.0/simRate;
    slidePos = 1000;
    dropPos = -1000;
    slidePosCmdPrev = 1000;
    dropPosCmdPrev = -1000;
    slideStop = 1;
    dropStop = 1;
}

void RobotSim::drive(double linV, double angV)
{
    heading = heading + angV*dt;
    xPos = xPos + linV*cos(heading*DEG2RAD)*dt;
    yPos = yPos + linV*sin(heading*DEG2RAD)*dt;
}

void RobotSim::teleport(double teleX, double teleY, double teleHeading)
{
    xPos = teleX;
    yPos = teleY;
    heading = teleHeading;
}

void RobotSim::runGrabber(int slidePosCmd, int dropPosCmd, int slideStopCmd, int dropStopCmd)
{
    if(slidePosCmd!=slidePosCmdPrev && slideStopCmd==0) slideStop = 0;
    if(dropPosCmd!=dropPosCmdPrev && slideStopCmd==0) dropStop = 0;
    if(slideStop==0)
    {
        if(fabs(slidePos-slidePosCmd)<=(int)round(slideSpeed_*dt)) slidePos = slidePosCmd;
        if(slidePosCmd > slidePos) slidePos += (int)round(slideSpeed_*dt);
        else if(slidePosCmd < slidePos) slidePos -= (int)round(slideSpeed_*dt);
        if(slidePos==slidePosCmd || slideStopCmd) slideStop = 1;
    }
    if(dropStop==0)
    {
        if(fabs(dropPos-dropPosCmd)<=(int)round(dropSpeed_*dt)) dropPos = dropPosCmd;
        if(dropPosCmd > dropPos) dropPos += (int)round(dropSpeed_*dt);
        else if(dropPosCmd < dropPos) dropPos -= (int)round(dropSpeed_*dt);
        if(dropPos==dropPosCmd || dropStopCmd) dropStop = 1;
    }
    if(slideStopCmd) slidePosCmdPrev = slidePos;
    else slidePosCmdPrev = slidePosCmd;
    if(dropStopCmd) dropPosCmdPrev = dropPos;
    else dropPosCmdPrev = dropPosCmd;
}
