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
