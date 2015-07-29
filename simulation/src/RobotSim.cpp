#include <simulation/RobotSim.h>

RobotSim::RobotSim(double initX, double initY, double initHeading, double simRate)
{
	teleport(initX, initY, initHeading);
    dt_ = 1.0/simRate;
}

void RobotSim::drive(double linV, double angV)
{
	heading = heading + angV*dt_;
	xPos = xPos + linV*cos(heading*DEG2RAD)*dt_;
	yPos = yPos + linV*sin(heading*DEG2RAD)*dt_;
}

void RobotSim::teleport(double teleX, double teleY, double teleHeading)
{
    xPos = teleX;
    yPos = teleY;
}
