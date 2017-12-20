/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

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
	const float distanceGain = 0.00165; // value/m
	const float terrainGain = 0.02; // value/terrainHazardIndex
	const float riskGain = 0.1;
    // Methods
    bool runProc();
};

#endif // NEXT_BEST_REGION_H
