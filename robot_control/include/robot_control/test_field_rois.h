#ifndef TEST_FIELD_ROIS_H
#define TEST_FIELD_ROIS_H

ROI.e = 105.0 - satMapStartE; // precached (0)
ROI.s = 35.0 - satMapStartS;
ROI.sampleProb = 1.0;
ROI.sampleSig = 10.0;
ROI.radialAxis = 9.5;
ROI.tangentialAxis = 9.5;
ROI.allocatedTime = 600.0;
ROI.highRisk = 0;
ROI.hardLockout = false;
ROI.roiGroup = 0;
ROI.whiteProb = 1.0;
ROI.silverProb = 1.0;
ROI.blueOrPurpleProb = 0.0;
ROI.pinkProb = 0.0;
ROI.redProb = 0.0;
ROI.orangeProb = 0.0;
ROI.yellowProb = 0.0;
regionsOfInterest.push_back(ROI);
ROI.e = 95.0 - satMapStartE; // 1
ROI.s = 90.0 - satMapStartS;
ROI.sampleProb = 0.5;
ROI.sampleSig = 1.0;
ROI.radialAxis = 9.5;
ROI.tangentialAxis = 9.5;
ROI.allocatedTime = 210.0;
ROI.highRisk = 0;
ROI.hardLockout = false;
ROI.roiGroup = 1;
ROI.whiteProb = 0.2;
ROI.silverProb = 0.2;
ROI.blueOrPurpleProb = 0.2;
ROI.pinkProb = 0.07;
ROI.redProb = 0.07;
ROI.orangeProb = 0.07;
ROI.yellowProb = 0.07;

#endif // TEST_FIELD_ROIS_H
