#include <robot_control/mission_planning.h>

MissionPlanning::MissionPlanning()
{
    execActionClient = nh.serviceClient<messages::ExecAction>("control/exec/actionin");
	navSub = nh.subscribe<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout", 1, &MissionPlanning::navCallback_, this);
    ExecActionEndedSub = nh.subscribe<messages::ExecActionEnded>("control/exec/actionended", 1, &MissionPlanning::ExecActionEndedCallback_, this);
    intermediateWaypointsClient = nh.serviceClient<robot_control::IntermediateWaypoints>("/control/safepathing/intermediatewaypoints");
    reqROIClient = nh.serviceClient<robot_control::RegionsOfInterest>("/control/mapmanager/regionsofinterest");
    modROIClient = nh.serviceClient<robot_control::ModifyROI>("/control/mapmanager/modifyroi");
    searchMapClient = nh.serviceClient<robot_control::SearchMap>("/control/mapmanager/searchlocalmap");
    randomSearchWaypointsClient = nh.serviceClient<robot_control::RandomSearchWaypoints>("/control/mapmanager/randomsearchwaypoints");
    nb1Sub = nh.subscribe<messages::nb1_to_i7_msg>("hw_interface/nb1in/nb1in", 1, &MissionPlanning::nb1Callback_, this);
    collisionSub = nh.subscribe<messages::CollisionOut>("lidar/collisiondetectionout/collisiondetectionout", 1, &MissionPlanning::collisionCallback_, this);
    execInfoSub = nh.subscribe<messages::ExecInfo>("control/exec/info", 1, &MissionPlanning::execInfoCallback_, this);
    cvSamplesSub = nh.subscribe<messages::CVSamplesFound>("vision/samplesearch/samplesearchout", 1, &MissionPlanning::cvSamplesCallback_, this);
    collisionInterruptTrigger = false;
    inSearchableRegion = false;
    possessingSample = false;
    possibleSample = false;
    definiteSample = false;
    sampleDataActedUpon = false;
    sampleInCollectPosition = false;
    confirmedPossession = false;
    atHome = false;
    inDepositPosition = false;
    missionEnded = false;
    multiProcLockout = false;
    lockoutSum = 0;
    initComplete = false;
    pauseStarted = false;
    robotStatus.pauseSwitch = true;
    execDequeEmpty = true;
    avoidLockout = false;
    roiKeyframed = false;
    execLastProcType = __depositSample__;
    execLastSerialNum = 99;
    allocatedROITime = 480.0; // sec == 8 min
    collisionInterruptThresh = 1.0; // m
    avoid.reg(__avoid__);
    nextBestRegion.reg(__nextBestRegion__); // consider polymorphic constructor
    //searchRegion.reg(__searchRegion__);
    approach.reg(__approach__);
    collect.reg(__collect__);
    confirmCollect.reg(__confirmCollect__);
    goHome.reg(__goHome__);
    depositApproach.reg(__depositApproach__);
    depositSample.reg(__depositSample__);
    depositSample.sendOpen(); // Make sure the grabber is open initially
    //procsToExecute.resize(NUM_PROC_TYPES);
    samplesCollected = 0;
    currentROIIndex = 0;
    for(int i=0; i<NUM_PROC_TYPES; i++)
    {
        procsToExecute[i] = false;
        procsToInterrupt[i] = false;
        procsBeingExecuted[i] = false;
        //procsToExecute.push_back(false);
        //procsToInterrupt.push_back(false);
        //procsBeingExecuted.push_back(false);
        ROS_INFO("procsToExecute.at(i) = %d",procsToExecute[i]);
        //ROS_INFO("procsToInterrupt.at(i) = %d",procsToInterrupt.at(i));
        //ROS_INFO("procsBeingExecuted.at(i) = %d",procsBeingExecuted.at(i));
    }
}

void MissionPlanning::run()
{
    //ROS_DEBUG("before evalConditions");
    ROS_INFO("=========================================");
    ROS_INFO("possessingSample = %i",possessingSample);
    ROS_INFO("possibleSample = %i",possibleSample);
    ROS_INFO("definiteSample = %i",definiteSample);
    ROS_INFO("sampleDataActedUpon = %i",sampleDataActedUpon);
    ROS_INFO("sampleInCollectPosition = %i",sampleInCollectPosition);
    ROS_INFO("confirmedPossession = %i",confirmedPossession);
    ROS_INFO("atHome = %i",atHome);
    ROS_INFO("inDepositPosition = %i",inDepositPosition);

   /* ROS_INFO("\n");
    bestSample.distance = 1.934814;
    bestSample.bearing = 3.434462;
    bestSample.confidence = 1000;
    distanceToDrive = bestSample.distance - distanceToGrabber - blindDriveDistance;
    if(distanceToDrive > 4.0) distanceToDrive = 4.0;
    angleToTurn = bestSample.bearing;
    approach.computeExpectedSampleLocation();
    ROS_INFO("expectedSampleDistance = %f",expectedSampleDistance);
    ROS_INFO("expectedSampleAngle = %f",expectedSampleAngle);*/

    evalConditions_();
    //ROS_DEBUG("after evalConditions");
    ROS_DEBUG("robotStatus.pauseSwitch = %i",robotStatus.pauseSwitch);
    if(robotStatus.pauseSwitch) runPause_();
    else runProcesses_();
    std::printf("\n");
}

void MissionPlanning::avoidObstacle_()
{
    //commandedAvoidObstacle = true;
    //sendDriveRel_(/*avoidMsg*/);
}

void MissionPlanning::returnHome_()
{
/*    commandedReturnHome = true;
    numWaypointsToTravel = 1;
    clearAndResizeWTT_();
    waypointsToTravel.at(0).x = homeX;
    waypointsToTravel.at(0).y = homeY;
    callIntermediateWaypoints_();
    sendDriveGlobal_();*/
}

void MissionPlanning::deposit_()
{

}

void MissionPlanning::acquire_()
{

}

void MissionPlanning::examine_()
{

}

void MissionPlanning::planRegionPath_()
{
/*    // *** Change this function to focus on path planning using ant colony within a region ***
	// Decision on how to set thresholds, based on which type of samples are being searched for. Constant for now
	woiSrv.request.easyThresh = 200;
	woiSrv.request.medThresh = 200;
	woiSrv.request.hardThresh = 200;
	// ********************************
	if(woiSrv.request.easyThresh > 1000) includeEasy = 0;
	else includeEasy = 1;
	if(woiSrv.request.medThresh > 1000) includeMed = 0;
	else includeMed = 1;
	if(woiSrv.request.hardThresh > 1000) includeHard = 0;
	else includeHard = 1;
	if(woiClient.call(woiSrv)) ROS_DEBUG("woi service call successful");
	else ROS_ERROR("woi service call unsuccessful");
	numWaypointsToPlan = woiSrv.response.waypointArray.size() + 1;
	numWaypointsToTravel = numWaypointsToPlan - 1;
	waypointsToPlan.resize(numWaypointsToTravel);
	ROS_DEBUG("before waypointArray copy");
	std::copy(woiSrv.response.waypointArray.begin(), woiSrv.response.waypointArray.end(), waypointsToPlan.begin());
	ROS_DEBUG("after waypointArray copy");
	currentLocation.x = 0.0; // Replace with actual current x and y
	currentLocation.y = 0.0; // ***
	waypointsToPlan.push_back(currentLocation);
	ROS_DEBUG("before antColony_()");
	antColony_();
	ROS_DEBUG("after antColony_()");
    callIntermediateWaypoints_();
    sendDriveGlobal_();*/
}

void MissionPlanning::chooseRegion_()
{
    
}

void MissionPlanning::init_()
{

}

void MissionPlanning::evalConditions_()
{
    if(multiProcLockout)
    {
        multiProcLockout = true;
        robotStatus.pauseSwitch = true;
        ROS_FATAL_THROTTLE(3,"tried to execute multiple procedures..........");
    }
    else
    {
        //for(int i; i<NUM_PROC_TYPES; i++) {procsToExecute.at(i) = false; procsToInterrupt.at(i) = false;}
        if(collisionMsg.collision!=0 && !execInfoMsg.turnFlag && !execInfoMsg.stopFlag && !avoidLockout && !missionEnded) // Avoid
        {
            for(int i=1; i<NUM_PROC_TYPES; i++) procsToInterrupt[i] = procsBeingExecuted[i];
            procsToInterrupt[__avoid__] = false;
            if(!procsBeingExecuted[__avoid__]) procsToExecute[__avoid__] = true;
            else if((collisionMsg.distance_to_collision <= collisionInterruptThresh) && procsBeingExecuted[__avoid__]) procsToInterrupt[__avoid__] = true;
            ROS_INFO("to execute avoid");
        }
        calcNumProcsBeingExec_();
        if(numProcsBeingExec==0 && !possessingSample && !(possibleSample || definiteSample) && !inSearchableRegion && !missionEnded) // Next Best Region
        {
            procsToExecute[__nextBestRegion__] = true;
            ROS_INFO("to execute nextBestRegion");
        }
        calcNumProcsBeingExec_();
        /*if(numProcsBeingExec==0 && !possessingSample && !(possibleSample || definiteSample) && inSearchableRegion && !missionEnded) // Search Region
        {

        }
        calcNumProcsBeingExec_();*/
        /*if(numProcsBeingExec==0 && !possessingSample && possibleSample && !definiteSample && !sampleDataActedUpon && !missionEnded) // Examine
        {
            sampleDataActedUpon = true;
            procsToExecute[__examine__] = true;
            ROS_INFO("to execute examine");
        }*/
        calcNumProcsBeingExec_();
        if(numProcsBeingExec==0 && !possessingSample && definiteSample && !sampleInCollectPosition && !missionEnded) // Approach
        {
            procsToExecute[__approach__] = true;
            ROS_INFO("to execute approach");
        }
        calcNumProcsBeingExec_();
        if(numProcsBeingExec==0 && sampleInCollectPosition && !possessingSample && !missionEnded) // Collect
        {
            procsToExecute[__collect__] = true;
            ROS_INFO("to execute collect");
        }
        calcNumProcsBeingExec_();
        if(numProcsBeingExec==0 && possessingSample && !confirmedPossession && !missionEnded) // Confirm Collect
        {
            procsToExecute[__confirmCollect__] = true;
            ROS_INFO("to execute confirmCollect");
        }
        calcNumProcsBeingExec_();
        if(numProcsBeingExec==0 && possessingSample && confirmedPossession && !atHome && !missionEnded) // Go Home
        {
            procsToExecute[__goHome__] = true;
            ROS_INFO("to execute goHome");
        }
        calcNumProcsBeingExec_();
        if(numProcsBeingExec==0 && possessingSample && confirmedPossession && atHome && !inDepositPosition && !missionEnded) // Deposit Approach
        {
            procsToExecute[__depositApproach__] = true;
            ROS_INFO("to execute depositApproach");
        }
        calcNumProcsBeingExec_();
        if(numProcsBeingExec==0 && possessingSample && confirmedPossession && atHome && inDepositPosition && !missionEnded) // Deposit Sample
        {
            procsToExecute[__depositSample__] = true;
            ROS_INFO("to execute depositSample");
        }

        // *************** Multi Proc Lockout for testing *************************
        lockoutSum = 0;
        for(int i=0; i<NUM_PROC_TYPES; i++) if(procsToExecute[i] && !procsToInterrupt[i]) lockoutSum++;
        if(lockoutSum>1) multiProcLockout = true;
        else multiProcLockout = false;
        if(multiProcLockout)
        {
            robotStatus.pauseSwitch = true;
            ROS_FATAL("tried to execute multiple procedures..........");
        }
        // *************************************************************************
    }
}

void MissionPlanning::runProcesses_()
{
    if(pauseStarted == true) pause.sendUnPause();
    pauseStarted = false;
    nextBestRegion.run();
    //searchClosestRegion.run();
    avoid.run();
    approach.run();
    collect.run();
    confirmCollect.run();
    goHome.run();
    depositApproach.run();
    depositSample.run();
}

void MissionPlanning::runPause_()
{
    if(pauseStarted == false) pause.sendPause();
    pauseStarted = true;
}

void MissionPlanning::calcNumProcsBeingExec_()
{
    numProcsBeingExec = 0;
    for(int i=0; i<NUM_PROC_TYPES; i++) 
    {
	if(procsBeingExecuted[i]) numProcsBeingExec++;
    }
}

void MissionPlanning::updateSampleFlags_()
{
    possibleSample = false;
    definiteSample = false;
    for(int i=0; i<cvSamplesFoundMsg.sampleList.size(); i++)
    {
        if(cvSamplesFoundMsg.sampleList.at(i).confidence >= possibleSampleConfThresh) possibleSample = true;
        if(cvSamplesFoundMsg.sampleList.at(i).confidence >= definiteSampleConfThresh) definiteSample = true;
    }
}

void MissionPlanning::navCallback_(const messages::NavFilterOut::ConstPtr& msg)
{
	robotStatus.xPos = msg->x_position;
	robotStatus.yPos = msg->y_position;
	robotStatus.heading = msg->heading;
	robotStatus.bearing = msg->bearing;
}

void MissionPlanning::ExecActionEndedCallback_(const messages::ExecActionEnded::ConstPtr &msg)
{
    execDequeEmpty = msg->dequeEmpty;
    execLastProcType = static_cast<PROC_TYPES_T>(msg->procType);
    execLastSerialNum = msg->serialNum;
}

void MissionPlanning::nb1Callback_(const messages::nb1_to_i7_msg::ConstPtr& msg)
{
    if(msg->pause_switch==0) robotStatus.pauseSwitch = false;
    else robotStatus.pauseSwitch = true;
}

void MissionPlanning::collisionCallback_(const messages::CollisionOut::ConstPtr &msg)
{
    collisionMsg = *msg;
}

void MissionPlanning::execInfoCallback_(const messages::ExecInfo::ConstPtr &msg)
{
    execInfoMsg = *msg;
}

void MissionPlanning::cvSamplesCallback_(const messages::CVSamplesFound::ConstPtr &msg)
{
    cvSamplesFoundMsg = *msg;
    updateSampleFlags_();
    sampleDataActedUpon = false;
}
