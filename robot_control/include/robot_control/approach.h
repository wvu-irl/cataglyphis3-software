#ifndef APPROACH_H
#define APPROACH_H
#include "procedure.h"

enum APPROACH_STEP_T {_computeManeuver, _performManeuver};

class Approach : public Procedure
{
public:
	// Members
	float distanceToDrive; // m
	float angleToTurn; // deg
	APPROACH_STEP_T step;
	uint8_t sampleTypeMux;
	const float maxDriveDistance = 4.0; // m
	int backUpCount = 0;
	const int maxBackUpCount = 1;
	const float backUpDistance = -0.5;
	bool approachableSample;
	unsigned int numSampleCandidates;
	std::vector<int> sampleValues;
	int bestSampleValue;
	float expectedSampleDistance;
	float expectedSampleAngle;
	const int sampleConfidenceGain = 1000;
	const int sampleDistanceToExpectedGain = 1000;
	const int approachValueThreshold = 700;
	bool commandedSearch;
	messages::CVSampleProps highestConfSample;
	// Methods
	Approach(); // constructor
	bool runProc();
	void computeSampleValues();
	void computeExpectedSampleLocation();
	void findHighestConfSample();
};

#endif // APPROACH_H
