#ifndef ROBOT_SIM_H
#define ROBOT_SIM_H
#include <math.h>

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
	// Methods
	RobotSim(double initX, double initY, double initHeading, double simRate); // Constructor
	void drive(double linV, double angV); // Drive robot using linear and angular velocities as input. Arg units: m/s, deg/s
	void teleport(double teleX, double teleY, double teleHeading); // Teleport robot to location. Arg units: m, m, deg
	void runGrabber(int slidePosCmd, int dropPosCmd, int slideStopCmd, int dropStopCmd); // Manage grabber operations
private:
	double dt_ = 0.05; // Default of 20 Hz
	const double slideSpeed_ = 2000.0/5.0; // full range of -1000 to 1000 in 5 seconds
	const double dropSpeed_ = 2000.0/5.0; // full range of -1000 to 1000 in 5 seconds
};

#endif // ROBOT_SIM_H
