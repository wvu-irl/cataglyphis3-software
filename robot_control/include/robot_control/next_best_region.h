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
	int roiSearchedSum;
	bool tempGoHome;
	std::vector<float> terrainHazard;
	const float hazardCorridorWidth = 2.0; // m
	const float numHazardsPerDistanceToTerrainHazardGain = 1.0; // terrainHzardIndex/(numHazards/m)
	const float sampleProbGain = 1.0; // value/conf
	const float sampleSigGain = 0.1; // value/sample significance
	const float distanceGain = 0.02; // value/m
	const float terrainGain = 0.2; // value/terrainHazardIndex
    // Methods
    bool runProc();
};

#endif // NEXT_BEST_REGION_H
