#include <robot_control/confirm_collect.h>

bool ConfirmCollect::runProc()
{
    //ROS_INFO("confirmCollectState = %i", state);
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		sampleTypeMux = 0;
		//sampleTypeMux = (1 << (static_cast<uint8_t>(bestSample.type) & 255)) & 255;
		//ROS_INFO("confirmCollect sampleTypeMux = %u",sampleTypeMux);
		//sendSearch(sampleTypeMux);
		expectedSampleAngle = 0.0;
		expectedSampleDistance = distanceToGrabber - backUpDistance;
        sendDriveRel(backUpDistance, 0.0, false, 0.0, false, false);
        sendSearch(252); // 124 = b11111100 -> cached = 1; purple = 1; red = 1; blue = 1; silver = 1; brass = 1; confirm = 0; save = 0;
		state = _exec_;
		break;
	case _exec_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		if(cvSamplesFoundMsg.procType==this->procType && cvSamplesFoundMsg.serialNum==this->serialNum)
		{
			computeSampleValuesWithExpectedDistance();
            ROS_INFO("confirmCollect bestSampleValue = %f",bestSampleValue);
			if(bestSampleValue < confirmCollectValueThreshold) noSampleOnGround = true;
			else noSampleOnGround = false;
			if(noSampleOnGround)
			{
                ROS_INFO("confirmCollect success, no sample on ground");
				confirmedPossession = true;
                inSearchableRegion = false;
                // Delete searchLocalMap
                searchMapSrv.request.createMap = false;
                searchMapSrv.request.deleteMap = true;
                if(searchMapClient.call(searchMapSrv)) ROS_DEBUG("searchMap service call successful");
                else ROS_ERROR("searchMap service call unsuccessful");
                // Set ROI to searched
                modROISrv.request.setHardLockoutROI = false;
                modROISrv.request.hardLockoutROIState = false;
                modROISrv.request.modROIIndex = currentROIIndex;
                modROISrv.request.addNewROI = false;
                modROISrv.request.deleteROI = false;
                modROISrv.request.setSampleProps = true;
                modROISrv.request.sampleProb = sampleFoundNewROIProb;
                modROISrv.request.sampleSig = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).sampleSig;
                if(modROIClient.call(modROISrv)) ROS_DEBUG("modify ROI service call successful");
                else ROS_ERROR("modify ROI service call unsuccessful");
                roiKeyframed = false;
				state = _finish_;
			}
			else
			{
                ROS_INFO("confirmCollect failed, sample still on ground");
                confirmCollectFailedCount++;
                if(confirmCollectFailedCount>=confirmCollectFailedLimit)
                {
                    ROS_INFO("confirmCollect over failed limit");
                    possibleSample = false;
                    definiteSample = false;
                    confirmCollectFailedCount = 0;
                    examineCount = 0;
                    backUpCount = 0;
                }
				confirmedPossession = false;
				possessingSample = false;
				sendOpen();
				state = _finish_;
			}
		}
		else state = _exec_;
		break;
	case _interrupt_:
		avoidLockout = false;
		sampleDataActedUpon = true;
		sampleInCollectPosition = false;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _init_;
		break;
	case _finish_:
		avoidLockout = false;
		sampleDataActedUpon = true;
		sampleInCollectPosition = false;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		state = _init_;
		break;
	}
}
