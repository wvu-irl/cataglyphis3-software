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
        //sampleTypeMux = (1 << (static_cast<uint8_t>(highestConfSample.type) & 255)) & 255;
		//ROS_INFO("confirmCollect sampleTypeMux = %u",sampleTypeMux);
		//sendSearch(sampleTypeMux);
		expectedSampleAngle = 0.0;
		expectedSampleDistance = distanceToGrabber - backUpDistance;
        sendDriveRel(backUpDistance, 0.0, false, 0.0, false, false);
        sendSearch(regionsOfInterestSrv.response.ROIList.at(currentROIIndex).whiteProb,
                   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).silverProb,
                   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).blueOrPurpleProb,
                   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).pinkProb,
                   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).redProb,
                   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).orangeProb,
                   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).yellowProb);
		state = _exec_;
        resetQueueEmptyCondition();
		break;
	case _exec_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        if(searchEnded() || queueEmptyTimedOut)
		{
			computeSampleValuesWithExpectedDistance(false);
            ROS_INFO("confirmCollect sampleDistanceAdjustedConf = %f",sampleDistanceAdjustedConf);
            ROS_INFO("confirmCollect highestConfSample.conf = %f",highestConfSample.confidence);
            if((sampleDistanceAdjustedConf < confirmCollectValueThreshold) || ((sampleDistanceAdjustedConf >= confirmCollectValueThreshold) && !sampleHistoryTypeMatch(highestConfSample.types))) noSampleOnGround = true;
			else noSampleOnGround = false;
			if(noSampleOnGround)
			{
                ROS_INFO("confirmCollect success, no sample on ground");
				confirmedPossession = true;
                inSearchableRegion = false;
                possibleSample = false;
                definiteSample = false;
                reorientCount = 0;
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
                if(currentROIIndex==0) modROISrv.request.sampleProb = 0.0;
                else modROISrv.request.sampleProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).sampleProb*sampleFoundNewROIProbMultiplier;
                modROISrv.request.sampleSig = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).sampleSig;
                modROISrv.request.sampleSig = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).sampleSig;
                modROISrv.request.whiteProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).whiteProb;
                modROISrv.request.silverProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).silverProb;
                modROISrv.request.blueOrPurpleProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).blueOrPurpleProb;
                modROISrv.request.pinkProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).pinkProb;
                modROISrv.request.redProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).redProb;
                modROISrv.request.orangeProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).orangeProb;
                modROISrv.request.yellowProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).yellowProb;
                modROISrv.request.editGroup = true;
                if(modROIClient.call(modROISrv)) ROS_DEBUG("modify ROI service call successful");
                else ROS_ERROR("modify ROI service call unsuccessful");
                roiKeyframed = false;
				state = _finish_;
			}
			else
			{
                ROS_INFO("confirmCollect failed, sample still on ground");
                confirmCollectFailedCount++;
                if(reorientCount>=reorientLimit && confirmCollectFailedCount>=confirmCollectFailedLimit)
                {
                    ROS_INFO("confirmCollect over reorient limit");
                    possibleSample = false;
                    definiteSample = false;
                    performReorient = false;
                    confirmCollectFailedCount = 0;
                    examineCount = 0;
                    backUpCount = 0;
                    reorientCount = 0;
                }
                else if(confirmCollectFailedCount>=confirmCollectFailedLimit)
                {
                    ROS_INFO("confirmCollect over failed limit");
                    possibleSample = false;
                    definiteSample = false;
                    performReorient = true;
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
        serviceQueueEmptyCondition();
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
