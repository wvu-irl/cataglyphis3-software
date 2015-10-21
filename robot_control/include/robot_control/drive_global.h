#ifndef DRIVE_GLOBAL_H
#define DRIVE_GLOBAL_H
#include "action.h"

class DriveGlobal : public Action
{
public:
	void init();
	int run();
private:
	float desiredX_;
	float desiredY_;
	float vMax_;
	float rMax_;
	float desiredEndHeading_;
	bool endHeading_;
	float distanceToDrive_;
	float angleToTurn_;
	float xErr_;
	float yErr_;
	float uXDes_;
	float uYDes_;
	float uXAct_;
	float uYAct_;
	float newHeadingSign_;
	void calculatePath_();
};

#endif // DRIVE_GLOBAL_H
