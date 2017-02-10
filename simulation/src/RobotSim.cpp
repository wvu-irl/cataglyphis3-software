#include <simulation/RobotSim.h>

RobotSim::RobotSim(double initX, double initY, double initHeading, double simRate)
{
	teleport(initX, initY, initHeading);
    normalSpeedDT = 1.0/simRate;
    dt = normalSpeedDT;
    slidePos = 1000;
    dropPos = -1000;
    slidePosCmdPrev = 1000;
    dropPosCmdPrev = -1000;
    slideStop = 1;
    dropStop = 1;
    grabAttempt = false;
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
    if(dropPos == GRABBER_DROPPED && slidePos > GRABBER_CLOSED && slidePosCmd == GRABBER_CLOSED) grabAttempt = true;
    if(slideStopCmd) slidePosCmdPrev = slidePos;
    else slidePosCmdPrev = slidePosCmd;
    if(dropStopCmd) dropPosCmdPrev = dropPos;
    else dropPosCmdPrev = dropPosCmd;
}
