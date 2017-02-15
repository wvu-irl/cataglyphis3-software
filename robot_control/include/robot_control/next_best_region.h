#ifndef NEXT_BEST_REGION_H
#define NEXT_BEST_REGION_H
#include "procedure.h"

class NextBestRegion : public Procedure
{
public:
    // Members
    int bestROINum;
	float roiValue;
	float bestROIValue;
	int roiHardLockoutSum;
	bool tempGoHome;
	const float distanceShortOfROI = 5.0; // m
	const float maxDistanceToSearchRegion = 5.0; // m
	float distanceToRegion;
	float angleToROI;
	std::vector<float> terrainHazard;
	const float negValueIncrement = 0.2;
	const float maxCoercedNegValue = 0.2;
	const float hazardCorridorWidth = 2.0; // m
	const float numHazardsPerDistanceToTerrainHazardGain = 1.0; // terrainHzardIndex/(numHazards/m)
	const float sampleProbGain = 1.0; // value/conf
	const float sampleSigGain = 0.1; // value/sample significance
	const float distanceGain = 0.00025; // value/m 0.00165
	const float terrainGain = 0.0025; // value/terrainHazardIndex 0.02
	const float riskGain = 0.1;
    // Methods
    bool runProc();
};

#endif // NEXT_BEST_REGION_H
