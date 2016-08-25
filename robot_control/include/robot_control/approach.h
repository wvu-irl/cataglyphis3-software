#ifndef APPROACH_H
#define APPROACH_H
#include "procedure.h"

enum APPROACH_STEP_T {_computeManeuver, _performManeuver};

class Approach : public Procedure
{
public:
	// Members
	APPROACH_STEP_T step;
	uint8_t sampleTypeMux;
	const float maxDriveDistance = 4.0; // m
	const int maxBackUpCount = 3;
	const int confirmCollectsFailedBeforeSideGrab = 100; // 1
	const float backUpDistance = -0.5;
	const float approachValueThreshold = 0.65;
	const float pitchCorrectionGain = 0.0;//0.02; // m/deg
	bool commandedSearch;
	bool approachableSample;
	float grabberDistanceTolerance;
	float grabberAngleTolerance;
	// Methods
	Approach(); // constructor
	bool runProc();
private:
	void computeManeuver_();
	bool sampleInPosition_();
};

#endif // APPROACH_H
