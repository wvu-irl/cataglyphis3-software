#ifndef ROBOT_SIM_H
#define ROBOT_SIM_H
#include<math.h>

#define RAD2DEG 180.0/3.14159
#define DEG2RAD 3.14159/180.0

class RobotSim
{
public:
	// Members
	double xPos; // m
	double yPos; // m
	double heading; // deg
	// Methods
	RobotSim(double initX, double initY, double initHeading, double simRate); // Constructor
	void drive(double linV, double angV); // Drive robot using linear and angular velocities as input. Arg units: m/s, deg/s
	void teleport(double teleX, double teleY, double teleHeading); // Teleport robot to location. Arg units: m, m, deg
private:
	double dt_ = 0.05; // Default of 20 Hz
};

#endif // ROBOT_SIM_H
