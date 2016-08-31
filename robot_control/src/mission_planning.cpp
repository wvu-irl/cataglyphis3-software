#include <robot_control/mission_planning.h>

MissionPlanning::MissionPlanning()
{
    execActionClient = nh.serviceClient<messages::ExecAction>("control/exec/actionin");
    poseSub = nh.subscribe<messages::RobotPose>("/hsm/masterexec/globalpose", 1, &MissionPlanning::poseCallback_, this);
    ExecActionEndedSub = nh.subscribe<messages::ExecActionEnded>("control/exec/actionended", 1, &MissionPlanning::ExecActionEndedCallback_, this);
    intermediateWaypointsClient = nh.serviceClient<robot_control::IntermediateWaypoints>("/control/safepathing/intermediatewaypoints");
    reqROIClient = nh.serviceClient<robot_control::RegionsOfInterest>("/control/mapmanager/regionsofinterest");
    modROIClient = nh.serviceClient<robot_control::ModifyROI>("/control/mapmanager/modifyroi");
    searchMapClient = nh.serviceClient<robot_control::SearchMap>("/control/mapmanager/searchlocalmap");
    randomSearchWaypointsClient = nh.serviceClient<robot_control::RandomSearchWaypoints>("/control/mapmanager/randomsearchwaypoints");
    globalMapPathHazardsClient = nh.serviceClient<messages::MapPathHazards>("/control/mapmanager/globalmappathhazards");
    searchLocalMapPathHazardsClient = nh.serviceClient<messages::MapPathHazards>("/control/mapmanager/searchlocalmappathhazards");
    navControlClient = nh.serviceClient<messages::NavFilterControl>("/navigation/navigationfilter/control");
    nb1Sub = nh.subscribe<messages::nb1_to_i7_msg>("hw_interface/nb1in/nb1in", 1, &MissionPlanning::nb1Callback_, this);
    nb2Sub = nh.subscribe<messages::nb2_3_to_i7_msg>("hw_interface/nb2in/nb2in", 1, &MissionPlanning::nb2Callback_, this);
    collisionSub = nh.subscribe<messages::CollisionOut>("lidar/collisiondetectionout/collisiondetectionout", 1, &MissionPlanning::collisionCallback_, this);
    execInfoSub = nh.subscribe<messages::ExecInfo>("control/exec/info", 1, &MissionPlanning::execInfoCallback_, this);
    lidarFilterSub = nh.subscribe<messages::LidarFilterOut>("lidar/lidarfilteringout/lidarfilteringout", 1, &MissionPlanning::lidarFilterCallback_, this);
    hsmMasterStatusSub = nh.subscribe<messages::MasterStatus>("hsm/masterexecutive/masterstatus", 1, &MissionPlanning::hsmMasterStatusCallback_, this);
    cvSamplesSub = nh.subscribe<messages::CVSamplesFound>("vision/samplesearch/samplesearchout", 1, &MissionPlanning::cvSamplesCallback_, this);
    nextWaypointSub = nh.subscribe<messages::NextWaypointOut>("/control/exec/nextwaypoint", 1, &MissionPlanning::nextWaypointCallback_, this);
    navSub = nh.subscribe<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout", 1, &MissionPlanning::navCallback_, this);
    grabberStatusSub = nh.subscribe<messages::ExecGrabberStatus>("/control/exec/grabberstatus", 1, &MissionPlanning::grabberStatusCallback_, this);
    emergencyEscapeServ = nh.advertiseService("/control/missionplanning/emergencyescapetrigger", &MissionPlanning::emergencyEscapeCallback_, this);
    controlServ = nh.advertiseService("/control/missionplanning/control", &MissionPlanning::controlCallback_, this);
    infoPub = nh.advertise<messages::MissionPlanningInfo>("/control/missionplanning/info", 1);
    driveSpeedsPub = nh.advertise<robot_control::DriveSpeeds>("/control/missionplanning/drivespeeds", 1);
    voiceSay = new Voice;
    initialized = false;
    collisionInterruptTrigger = false;
    escapeCondition = false;
    performBiasRemoval = false;
    performHoming = false;
    inSearchableRegion = false;
    roiTimeExpired = false;
    roiOvertimeExpired = false;
    possessingSample = false;
    possibleSample = false;
    definiteSample = false;
    sampleDataActedUpon = false;
    sampleInCollectPosition = false;
    sideOffsetGrab = false;
    performReorient = false;
    confirmedPossession = false;
    atHome = false;
    homingUpdateFailed = false;
    performSafeMode = false;
    inDepositPosition = false;
    missionEnded = false;
    useDeadReckoning = false;
    multiProcLockout = false;
    lockoutSum = 0;
    initComplete = false;
    pauseStarted = false;
    robotStatus.pauseSwitch = true;
    execDequeEmpty = true;
    avoidLockout = true;
    escapeLockout = false;
    roiKeyframed = false;
    startSLAM = false;
    giveUpROI = false;
    possiblyLost = false;
    searchTimedOut = false;
    newSearchActionOnExec = false;
    tiltTooExtremeForBiasRemoval = false;
    navStopRequest = false;
    nb1Good = false;
    nb2Good = false;
    nb1Pause = true;
    nb2Pause = true;
    avoidCount = 0;
    prevAvoidCountDecXPos = robotStatus.xPos;
    prevAvoidCountDecYPos = robotStatus.yPos;
    execLastProcType = __depositSample__;
    execLastSerialNum = 99;
    allocatedROITime = 480.0; // sec == 8 min
    collisionInterruptThresh = 1.0; // m
    initialize.reg(__initialize__);
    emergencyEscape.reg(__emergencyEscape__);
    avoid.reg(__avoid__);
    biasRemoval.reg(__biasRemoval__);
    nextBestRegion.reg(__nextBestRegion__);
    searchRegion.reg(__searchRegion__);
    examine.reg(__examine__);
    approach.reg(__approach__);
    collect.reg(__collect__);
    confirmCollect.reg(__confirmCollect__);
    reorient.reg(__reorient__);
    goHome.reg(__goHome__);
    squareUpdate.reg(__squareUpdate__);
    depositApproach.reg(__depositApproach__);
    depositSample.reg(__depositSample__);
    safeMode.reg(__safeMode__);
    sosMode.reg(__sosMode__);
    //procsToExecute.resize(NUM_PROC_TYPES);
    samplesCollected = 0;
    currentROIIndex = 0;
    prevROIIndex = 99;
    examineCount = 0;
    backUpCount = 0;
    reorientCount = 0;
    confirmCollectFailedCount = 0;
    homingUpdatedFailedCount = 0;
    missionTime = 0.0;
    prevTime = ros::Time::now().toSec();
    missionStarted = false;
    timers[_roiTimer_] = new CataglyphisTimer<MissionPlanning>(&MissionPlanning::roiTimeExpiredCallback_, this);
    timers[_biasRemovalTimer_] = new CataglyphisTimer<MissionPlanning>(&MissionPlanning::biasRemovalTimerCallback_, this);
    timers[_homingTimer_] = new CataglyphisTimer<MissionPlanning>(&MissionPlanning::homingTimerCallback_, this);
    timers[_searchTimer_] = new CataglyphisTimer<MissionPlanning>(&MissionPlanning::searchTimerCallback_, this);
    timers[_biasRemovalActionTimeoutTimer_] = new CataglyphisTimer<MissionPlanning>(&MissionPlanning::biasRemovalActionTimerCallback_, this);
    timers[_queueEmptyTimer_] = new CataglyphisTimer<MissionPlanning>(&MissionPlanning::queueEmptyTimerCallback_, this);
    timers[_roiOvertimeTimer_] = new CataglyphisTimer<MissionPlanning>(&MissionPlanning::roiOvertimeTimerCallback_, this);
    timers[_roiTimer_]->stop();
    timers[_biasRemovalActionTimeoutTimer_]->setPeriod(biasRemovalActionTimeoutTime);
    timers[_searchTimer_]->setPeriod(searchTimeoutPeriod);
    timers[_searchTimer_]->stop();
    timers[_biasRemovalActionTimeoutTimer_]->stop();
    timers[_biasRemovalTimer_]->setPeriod(biasRemovalTimeoutPeriod);
    timers[_homingTimer_]->setPeriod(homingTimeoutPeriod);
    timers[_queueEmptyTimer_]->setPeriod(queueEmptyTimerPeriod);
    timers[_queueEmptyTimer_]->stop();
    timers[_roiOvertimeTimer_]->setPeriod(roiOvertimePeriod);
    timers[_roiOvertimeTimer_]->stop();
    timers[_biasRemovalTimer_]->start();
    timers[_homingTimer_]->start();
    driveSpeedsMsg.vMax = defaultVMax;
    driveSpeedsMsg.rMax = defaultRMax;
    driveSpeedsMsgPrev.vMax = 0.0;
    driveSpeedsMsgPrev.rMax = 0.0;
    driveSpeedsPub.publish(driveSpeedsMsg);
    infoMsg.procsToExecute.resize(NUM_PROC_TYPES,0);
    infoMsg.procsToInterrupt.resize(NUM_PROC_TYPES,0);
    infoMsg.procsBeingExecuted.resize(NUM_PROC_TYPES,0);
    infoMsg.procsToResume.resize(NUM_PROC_TYPES,0);
    infoMsg.procStates.resize(NUM_PROC_TYPES,_init_);
    initialize.clearSampleHistory();
    srand(time(NULL));
    for(int i=0; i<NUM_PROC_TYPES; i++)
    {
        procsToExecute[i] = false;
        procsToInterrupt[i] = false;
        procsBeingExecuted[i] = false;
        procsToResume[i] = false;
        //procsToExecute.push_back(false);
        //procsToInterrupt.push_back(false);
        //procsBeingExecuted.push_back(false);
        //ROS_INFO("procsToExecute.at(i) = %d",procsToExecute[i]);
        //ROS_INFO("procsToInterrupt.at(i) = %d",procsToInterrupt.at(i));
        //ROS_INFO("procsBeingExecuted.at(i) = %d",procsBeingExecuted.at(i));
    }
}

void MissionPlanning::run()
{
	if(nb1Good && nb2Good) robotStatus.pauseSwitch = nb1Pause || nb2Pause;
	else if(nb1Good && !nb2Good) robotStatus.pauseSwitch = nb1Pause;
	else if(!nb1Good && nb2Good) robotStatus.pauseSwitch = nb2Pause;
	else robotStatus.pauseSwitch = true;
    ROS_INFO_THROTTLE(3,"Mission Planning running...");
    evalConditions_();
    ROS_DEBUG("robotStatus.pauseSwitch = %i",robotStatus.pauseSwitch);
    if(robotStatus.pauseSwitch || navStopRequest) runPause_();
    else runProcedures_();
    packAndPubInfoMsg_();
    if(missionStarted && !robotStatus.pauseSwitch) missionTime += ros::Time::now().toSec() - prevTime;
    prevTime = ros::Time::now().toSec();
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
        //ROS_INFO("searchTimerRunning = %i",timers[_searchTimer_]->running);
        //std::printf("\n");
        //ROS_INFO("=========================================");
        /*ROS_INFO("escapeCondition = %i",escapeCondition);
        ROS_INFO("escapeLockout = %i",escapeLockout);
        ROS_INFO("collisionCondition = %i",collisionMsg.collision);
        ROS_INFO("avoidLockout = %i",avoidLockout);
        ROS_INFO("performBiasRemoval = %i",performBiasRemoval);
        ROS_INFO("performHoming = %i",performHoming);
        ROS_INFO("inSearchableRegion = %i",inSearchableRegion);
        ROS_INFO("roiTimeExpired = %i",roiTimeExpired);
        ROS_INFO("possessingSample = %i",possessingSample);
        ROS_INFO("possibleSample = %i",possibleSample);
        ROS_INFO("definiteSample = %i",definiteSample);
        //ROS_INFO("sampleDataActedUpon = %i",sampleDataActedUpon);
        ROS_INFO("sampleInCollectPosition = %i",sampleInCollectPosition);
        ROS_INFO("confirmedPossession = %i",confirmedPossession);
        ROS_INFO("atHome = %i",atHome);
        ROS_INFO("homingUpdateFailed = %i",homingUpdateFailed);
        ROS_INFO("performSafeMode = %i",performSafeMode);
        ROS_INFO("inDepositPosition = %i",inDepositPosition);
        ROS_INFO("avoidCount = %u",avoidCount);
        ROS_INFO("examineCount = %u",examineCount);
        ROS_INFO("turnFlag = %i",execInfoMsg.turnFlag);
        ROS_INFO("stopFlag = %i",execInfoMsg.stopFlag);
        ROS_INFO("searchRegion.state = %i",(int)searchRegion.state);*/
        /*ROS_INFO("execDequeSize = %u",execInfoMsg.actionDequeSize);
        std::printf("actionDeque: (");
        for(int i=0; i<execInfoMsg.actionDequeSize; i++) std::printf("%i,",execInfoMsg.actionDeque[i]);
        std::printf(")\n");
        std::printf("actionFloat1: (");
        for(int i=0; i<execInfoMsg.actionDequeSize; i++) std::printf("%f,",execInfoMsg.actionFloat1[i]);
        std::printf(")\n");
        std::printf("actionFloat2: (");
        for(int i=0; i<execInfoMsg.actionDequeSize; i++) std::printf("%f,",execInfoMsg.actionFloat2[i]);
        std::printf(")\n");*/
        //for(int i=0; i<NUM_PROC_TYPES; i++)
        //for(int i; i<NUM_PROC_TYPES; i++) {procsToExecute.at(i) = false; procsToInterrupt.at(i) = false;}
        calcnumProcsBeingOrToBeExecOrRes_();
        //ROS_INFO("numProcsBeingOrToBeExecOrRes = %i", numProcsBeingOrToBeExecOrRes);
        shouldExecuteAvoidManeuver = false;
        if(numProcsBeingOrToBeExecOrRes==0 && !initialized && !robotStatus.pauseSwitch && !missionEnded)
        {
            procsToExecute[__initialize__] = true;
            ROS_INFO("to execute initialize");
            voiceSay->call("initialize");
        }
        if(escapeCondition && !execInfoMsg.stopFlag && !escapeLockout && !robotStatus.pauseSwitch && initialized && !missionEnded) // Emergency Escape
        {
            for(int i=0; i<NUM_PROC_TYPES; i++) procsToInterrupt[i] = procsBeingExecuted[i];
            if(procsBeingExecuted[__emergencyEscape__]) {procsToInterrupt[__emergencyEscape__] = true; ROS_INFO("to interrupt emergencyEscape"); voiceSay->call("interrupt emergency escape");}
            else {procsToExecute[__emergencyEscape__] = true; procsToInterrupt[__emergencyEscape__] = false; ROS_INFO("to execute emergencyEscape"); voiceSay->call("emergency escape");}
        }
        if(!escapeCondition && collisionMsg.collision!=0 && !execInfoMsg.turnFlag && !execInfoMsg.stopFlag && !avoidLockout && !robotStatus.pauseSwitch && initialized && !missionEnded) // Avoid
        {
            ROS_INFO("avoid case");
            avoidLockout = true;
            shouldExecuteAvoidManeuver = true;
            for(int i=0; i<NUM_PROC_TYPES; i++) procsToInterrupt[i] = procsBeingExecuted[i];
            procsToInterrupt[__avoid__] = false;
            if(procsToInterrupt[__emergencyEscape__]) avoid.interruptedEmergencyEscape = true; // If avoid occured during emergency escape, just treat the avoid maneuver as the offset drive in emergency escape
            else avoid.interruptedEmergencyEscape = false;
            if(procsToInterrupt[__nextBestRegion__] || procsToInterrupt[__searchRegion__] || procsToInterrupt[__goHome__]) // If avoid occured while driving to a waypoint globally (which occurs in these procedures), check if the remaining distance to the waypoint is small enough to just end the drive there
            {
                //ROS_INFO("was executing proc with driveGlobal");
                if(static_cast<ACTION_TYPE_T>(execInfoMsg.actionDeque[0]) == _driveGlobal)
                {
                    //ROS_INFO("was executing driveGlobal");
                    avoidRemainingWaypointDistance = hypot(execInfoMsg.actionFloat1[0] - robotStatus.xPos, execInfoMsg.actionFloat2[0] - robotStatus.yPos);
                    //ROS_INFO("avoidRemainingWaypointDistance = %f",avoidRemainingWaypointDistance);
                    if(avoidRemainingWaypointDistance <= minAvoidRemainingWaypointDistance && !execInfoMsg.actionBool3[0])
                    {
                        ROS_INFO("closer than avoid remaining waypoint distance. End drive here");
                        voiceSay->call("close to waypoint. end drive here");
                        avoid.sendDequeClearFront();
                        shouldExecuteAvoidManeuver = false;
                    }
                }
                /*robotStatus.pauseSwitch = true;
                pause.sendPause();
                std::cout << "press enter to continue" << std::endl;
                std::cin >> temp;
                robotStatus.pauseSwitch = false;
                pause.sendUnPause();*/
            }
            else if(procsToInterrupt[__examine__] || procsToInterrupt[__approach__])
            {
                ROS_INFO("avoid interrupted examine or approach");
                avoid.sendDequeClearAll();
                shouldExecuteAvoidManeuver = false;
                possibleSample = false;
                definiteSample = false;
            }
            if(shouldExecuteAvoidManeuver)
            {
                if(procsBeingExecuted[__avoid__])
                {
                    procsToInterrupt[__avoid__] = true;
                    avoid.interruptedAvoid = 1;
                    ROS_INFO("to interrput avoid");
                    voiceSay->call("interrupt avoid");
                }
                else
                {
                    procsToExecute[__avoid__] = true;
                    avoid.interruptedAvoid = 0;
                    ROS_INFO("to execute avoid");
                    voiceSay->call("avoid");
                }
            }
        }
        if(numProcsBeingOrToBeExecOrRes==0 && !possessingSample && !confirmedPossession && !(possibleSample || definiteSample) && !inSearchableRegion && !sampleInCollectPosition && !performReorient && !escapeCondition && !shouldExecuteAvoidManeuver && !performBiasRemoval && !performHoming && !homingUpdateFailed && !performSafeMode && initialized && !missionEnded) // Next Best Region
        {
            procsToExecute[__nextBestRegion__] = true;
            ROS_INFO("to execute nextBestRegion");
            voiceSay->call("next best begion");
            //VOICESAY("to execute nextBestRegion");
            /*robotStatus.pauseSwitch = true;
            pause.sendPause();
            std::cout << "press enter to continue" << std::endl;
            std::cin >> temp;
            robotStatus.pauseSwitch = false;
            pause.sendUnPause();*/
        }
        if(numProcsBeingOrToBeExecOrRes==0 && !possessingSample && !confirmedPossession && !(possibleSample || definiteSample) && inSearchableRegion && !sampleInCollectPosition && !performReorient && !escapeCondition && !shouldExecuteAvoidManeuver && !performBiasRemoval && !performSafeMode && initialized && !missionEnded) // Search Region
        {
            procsToExecute[__searchRegion__] = true;
            ROS_INFO("to execute searchRegion");
            voiceSay->call("search region");
            /*robotStatus.pauseSwitch = true;
            pause.sendPause();
            std::cout << "press enter to continue" << std::endl;
            std::cin >> temp;
            robotStatus.pauseSwitch = false;
            pause.sendUnPause();*/
        }
        if(numProcsBeingOrToBeExecOrRes==0 && performBiasRemoval && /*!(!possessingSample != !confirmedPossession) && !(possibleSample || definiteSample) && !sampleInCollectPosition && !performReorient && !inDepositPosition && */!escapeCondition && !shouldExecuteAvoidManeuver && !performSafeMode && initialized && !missionEnded) // Bias Removal
        {
            procsToExecute[__biasRemoval__] = true;
            ROS_INFO("to execute bias removal");
            voiceSay->call("bias removal");
        }
        if(numProcsBeingOrToBeExecOrRes==0 && !possessingSample && !confirmedPossession && possibleSample && !definiteSample && !performReorient && !performBiasRemoval && !escapeCondition && !shouldExecuteAvoidManeuver && !performSafeMode && initialized && !missionEnded) // Examine
        {
            procsToExecute[__examine__] = true;
            ROS_INFO("to execute examine");
            voiceSay->call("examine");
        }
        if(numProcsBeingOrToBeExecOrRes==0 && !possessingSample && !confirmedPossession && definiteSample && !sampleInCollectPosition && !performReorient && !performBiasRemoval && !escapeCondition && !shouldExecuteAvoidManeuver && !performSafeMode && initialized && !missionEnded) // Approach
        {
            procsToExecute[__approach__] = true;
            ROS_INFO("to execute approach");
            voiceSay->call("approach");
        }
        if(numProcsBeingOrToBeExecOrRes==0 && sampleInCollectPosition && !possessingSample && !confirmedPossession && !performReorient && !performBiasRemoval && !escapeCondition && !shouldExecuteAvoidManeuver && !performSafeMode && initialized && !missionEnded) // Collect
        {
            procsToExecute[__collect__] = true;
            ROS_INFO("to execute collect");
            voiceSay->call("collect");
        }
        if(numProcsBeingOrToBeExecOrRes==0 && possessingSample && !confirmedPossession && !performReorient && !performBiasRemoval && !escapeCondition && !shouldExecuteAvoidManeuver && !performSafeMode && initialized && !missionEnded) // Confirm Collect
        {
            procsToExecute[__confirmCollect__] = true;
            ROS_INFO("to execute confirmCollect");
            voiceSay->call("confirm collect");
        }
        if(numProcsBeingOrToBeExecOrRes==0 && performReorient && !(possibleSample || definiteSample) && !sampleInCollectPosition && !possessingSample && !confirmedPossession && !performBiasRemoval && !escapeCondition && !shouldExecuteAvoidManeuver && !performSafeMode && initialized && !missionEnded) // Reorient
        {
            procsToExecute[__reorient__] = true;
            ROS_INFO("to execute reorient");
            voiceSay->call("reorient");
        }
        if(numProcsBeingOrToBeExecOrRes==0 && ((possessingSample && confirmedPossession) || (performHoming && !sampleInCollectPosition && !possessingSample && !performReorient && !inSearchableRegion)) && !atHome && !homingUpdateFailed && !(possibleSample || definiteSample) && !performBiasRemoval && !escapeCondition && !shouldExecuteAvoidManeuver && !performSafeMode && initialized && !missionEnded) // Go Home
        {
            procsToExecute[__goHome__] = true;
            ROS_INFO("to execute goHome");
            voiceSay->call("go home");
        }
        if(numProcsBeingOrToBeExecOrRes==0 && homingUpdateFailed && !(possibleSample || definiteSample) && !sampleInCollectPosition && !performReorient && !inDepositPosition && !performBiasRemoval && !escapeCondition && !shouldExecuteAvoidManeuver && !performSafeMode && initialized && !missionEnded) // Square Update
        {
            procsToExecute[__squareUpdate__] = true;
            ROS_INFO("to execute square update");
            voiceSay->call("square update");
        }
        if(numProcsBeingOrToBeExecOrRes==0 && possessingSample && confirmedPossession && atHome && !inDepositPosition && !homingUpdateFailed && !performReorient && !performBiasRemoval && !escapeCondition && !shouldExecuteAvoidManeuver && !performSafeMode && initialized && !missionEnded) // Deposit Approach
        {
            procsToExecute[__depositApproach__] = true;
            ROS_INFO("to execute depositApproach");
            voiceSay->call("deposit approach");
        }
        if(numProcsBeingOrToBeExecOrRes==0 && possessingSample && confirmedPossession && atHome && inDepositPosition && !homingUpdateFailed && !performReorient && !performBiasRemoval && !escapeCondition && !shouldExecuteAvoidManeuver && !performSafeMode && initialized && !missionEnded) // Deposit Sample
        {
            procsToExecute[__depositSample__] = true;
            ROS_INFO("to execute depositSample");
            voiceSay->call("deposit sample");
        }
        if(numProcsBeingOrToBeExecOrRes==0 && performSafeMode && !escapeCondition && !shouldExecuteAvoidManeuver && initialized && !missionEnded)
        {
            procsToExecute[__safeMode__] = true;
            ROS_INFO("to execute safeMode");
            voiceSay->call("safe mode");
        }

        calcnumProcsBeingOrToBeExecOrRes_();
        if((numProcsBeingOrToBeExecOrRes==0 || numProcsToBeExecAndNotInterrupt>1) && initialized && !missionEnded)
        {
            initialized = true;
            performHoming = true;
            inSearchableRegion = false;
            if(timers[_roiTimer_]->running) timers[_roiTimer_]->stop();
            roiTimeExpired = false;
            if(timers[_roiOvertimeTimer_]->running) timers[_roiOvertimeTimer_]->stop();
            roiOvertimeExpired = false;
            possibleSample = false;
            definiteSample = false;
            sampleInCollectPosition = false;
            sideOffsetGrab = false;
            performReorient = false;
            atHome = false;
            homingUpdateFailed = false;
            performSafeMode = false;
            inDepositPosition = false;
            if(roiKeyframed)
            {
                searchMapSrv.request.createMap = false;
                searchMapSrv.request.deleteMap = true;
                if(searchMapClient.call(searchMapSrv)) ROS_DEBUG("searchMap service call successful");
                else ROS_ERROR("searchMap service call unsuccessful");
                roiKeyframed = false;
            }
        }

        // *************** Multi Proc Lockout for testing *************************
        /*lockoutSum = 0;
        for(int i=0; i<NUM_PROC_TYPES; i++) if(procsToExecute[i] && !procsToInterrupt[i]) lockoutSum++;
        if(lockoutSum>1) multiProcLockout = true;
        else multiProcLockout = false;
        if(multiProcLockout)
        {
            robotStatus.pauseSwitch = true;
            ROS_FATAL("tried to execute multiple procedures..........");
            voiceSay->call("tried to execute multiple procedures. tisk tisk.");
        }*/
        // *************************************************************************
    }
}

void MissionPlanning::runProcedures_()
{
    if(pauseStarted == true)
    {
        pause.sendUnPause();
        resumeTimers_();
        voiceSay->call("un pause");
        if(missionStarted==false) missionStarted = true;
    }
    pauseStarted = false;
    initialize.run();
    emergencyEscape.run();
    avoid.run();
    biasRemoval.run();
    nextBestRegion.run();
    searchRegion.run();
    examine.run();
    approach.run();
    collect.run();
    confirmCollect.run();
    reorient.run();
    goHome.run();
    squareUpdate.run();
    depositApproach.run();
    depositSample.run();
    safeMode.run();
    sosMode.run();
}

void MissionPlanning::runPause_()
{
    if(pauseStarted == false)
    {
        pause.sendPause();
        pauseAllTimers_();
        voiceSay->call("pause");
    }
    pauseStarted = true;
}

void MissionPlanning::pauseAllTimers_()
{
    for(int i=0; i<NUM_TIMERS; i++) if(timers[i]->running && (static_cast<TIMER_NAMES_T>(i) != _biasRemovalTimer_)) timers[i]->pause();
}

void MissionPlanning::resumeTimers_()
{
    for(int i=0; i<NUM_TIMERS; i++) if(timers[i]->running && (static_cast<TIMER_NAMES_T>(i) != _biasRemovalTimer_)) timers[i]->resume();
}

void MissionPlanning::serviceSearchTimer_()
{
    if(execInfoMsg.actionDeque[0] == _search && !timers[_searchTimer_]->running && !searchTimedOut && !newSearchActionOnExec) {timers[_searchTimer_]->start(); newSearchActionOnExec = true; ROS_INFO("start searchTimer");}
    else if(execInfoMsg.actionDeque[0] != _search) newSearchActionOnExec = false;
}

void MissionPlanning::calcnumProcsBeingOrToBeExecOrRes_()
{
    numProcsBeingOrToBeExecOrRes = 0;
    numProcsBeingOrToBeExec = 0;
    numProcsToBeExecAndNotInterrupt = 0;
    for(int i=0; i<NUM_PROC_TYPES; i++) 
    {
        if(procsBeingExecuted[i] || procsToExecute[i] || procsToResume[i]) numProcsBeingOrToBeExecOrRes++;
        if(procsBeingExecuted[i] || procsToExecute[i]) numProcsBeingOrToBeExec++;
        if(procsToExecute[i] && !procsToInterrupt[i]) numProcsToBeExecAndNotInterrupt++;
    }
}

/*void MissionPlanning::updateSampleFlags_()
{
    possibleSample = false;
    definiteSample = false;
    for(int i=0; i<cvSamplesFoundMsg.sampleList.size(); i++)
    {
        if(cvSamplesFoundMsg.sampleList.at(i).confidence >= possibleSampleConfThresh) possibleSample = true;
        if(cvSamplesFoundMsg.sampleList.at(i).confidence >= definiteSampleConfThresh) definiteSample = true;
    }
}*/

void MissionPlanning::packAndPubInfoMsg_()
{
    infoMsg.initialized = initialized;
    infoMsg.escapeCondition = escapeCondition;
    infoMsg.escapeLockout = escapeLockout;
    infoMsg.avoidLockout = avoidLockout;
    infoMsg.shouldExecuteAvoidManeuver = shouldExecuteAvoidManeuver;
    infoMsg.performBiasRemoval = performBiasRemoval;
    infoMsg.performHoming = performHoming;
    infoMsg.inSearchableRegion = inSearchableRegion;
    infoMsg.roiTimeExpired = roiTimeExpired;
    infoMsg.possessingSample = possessingSample;
    infoMsg.possibleSample = possibleSample;
    infoMsg.definiteSample = definiteSample;
    infoMsg.sampleInCollectPosition = sampleInCollectPosition;
    infoMsg.sideOffsetGrab = sideOffsetGrab;
    infoMsg.performReorient = performReorient;
    infoMsg.confirmedPossession = confirmedPossession;
    infoMsg.atHome = atHome;
    infoMsg.homingUpdateFailed = homingUpdateFailed;
    infoMsg.performSafeMode = performSafeMode;
    infoMsg.inDepositPosition = inDepositPosition;
    infoMsg.missionEnded = missionEnded;
    infoMsg.useDeadReckoning = useDeadReckoning;
    infoMsg.possiblyLost = possiblyLost;
    infoMsg.roiKeyframed = roiKeyframed;
    infoMsg.startSLAM = startSLAM;
    infoMsg.giveUpROI = giveUpROI;
    infoMsg.pause = robotStatus.pauseSwitch;
    infoMsg.numProcs = NUM_PROC_TYPES;
    infoMsg.samplesCollected = samplesCollected;
    infoMsg.avoidCount = avoidCount;
    infoMsg.examineCount = examineCount;
    infoMsg.backupCount = backUpCount;
    infoMsg.confirmCollectFailedCount = confirmCollectFailedCount;
    infoMsg.reorientCount = reorientCount;
    infoMsg.homingUpdatedFailedCount = homingUpdatedFailedCount;
    for(int i=0; i<NUM_PROC_TYPES; i++)
    {
        infoMsg.procsToExecute.at(i) = procsToExecute[i];
        infoMsg.procsToInterrupt.at(i) = procsToInterrupt[i];
        infoMsg.procsBeingExecuted.at(i) = procsBeingExecuted[i];
        infoMsg.procsToResume.at(i) = procsToResume[i];
        switch(static_cast<PROC_TYPES_T>(i))
        {
        case __initialize__:
            infoMsg.procStates.at(i) = initialize.state;
            break;
        case __emergencyEscape__:
            infoMsg.procStates.at(i) = emergencyEscape.state;
            break;
        case __avoid__:
            infoMsg.procStates.at(i) = avoid.state;
            break;
        case __biasRemoval__:
            infoMsg.procStates.at(i) = biasRemoval.state;
            break;
        case __nextBestRegion__:
            infoMsg.procStates.at(i) = nextBestRegion.state;
            break;
        case __searchRegion__:
            infoMsg.procStates.at(i) = searchRegion.state;
            break;
        case __examine__:
            infoMsg.procStates.at(i) = examine.state;
            break;
        case __approach__:
            infoMsg.procStates.at(i) = approach.state;
            break;
        case __collect__:
            infoMsg.procStates.at(i) = collect.state;
            break;
        case __confirmCollect__:
            infoMsg.procStates.at(i) = confirmCollect.state;
            break;
        case __reorient__:
            infoMsg.procStates.at(i) = reorient.state;
            break;
        case __goHome__:
            infoMsg.procStates.at(i) = goHome.state;
            break;
        case __squareUpdate__:
            infoMsg.procStates.at(i) = squareUpdate.state;
            break;
        case __depositApproach__:
            infoMsg.procStates.at(i) = depositApproach.state;
            break;
        case __depositSample__:
            infoMsg.procStates.at(i) = depositSample.state;
            break;
        case __safeMode__:
            infoMsg.procStates.at(i) = safeMode.state;
            break;
        case __sosMode__:
            infoMsg.procStates.at(i) = sosMode.state;
            break;
        }
    }
    infoMsg.collisionCondition = collisionMsg.collision;
    infoMsg.missionTime = missionTime;
    infoMsg.currentROIIndex = currentROIIndex;
    infoPub.publish(infoMsg);
}

void MissionPlanning::poseCallback_(const messages::RobotPose::ConstPtr& msg)
{
    robotStatus.xPos = msg->x;
    robotStatus.yPos = msg->y;
	robotStatus.heading = msg->heading;
    robotStatus.bearing = RAD2DEG*atan2(msg->y, msg->x);
    //if(msg->homingUpdated && !robotStatus.homingUpdated) avoid.sendWait(shortRecomputeActionWaitTime, true);
    robotStatus.homingUpdated = msg->homingUpdated;
    if(robotStatus.homingUpdated)
    {
        timers[_homingTimer_]->stop();
        timers[_homingTimer_]->start();
        //performHoming = false;
        //possiblyLost = false;
        //homingUpdateFailed = false;
    }
}

void MissionPlanning::ExecActionEndedCallback_(const messages::ExecActionEnded::ConstPtr &msg)
{
    execDequeEmpty = msg->dequeEmpty;
    execLastProcType = static_cast<PROC_TYPES_T>(msg->procType);
    execLastSerialNum = msg->serialNum;
}

void MissionPlanning::nb1Callback_(const messages::nb1_to_i7_msg::ConstPtr& msg)
{
    if(msg->pause_switch==0) nb1Pause = false;
    else nb1Pause = true;
}

void MissionPlanning::nb2Callback_(const messages::nb2_3_to_i7_msg::ConstPtr& msg)
{
    if(msg->pause_switch==0) nb2Pause = false;
    else nb2Pause = true;
}

void MissionPlanning::collisionCallback_(const messages::CollisionOut::ConstPtr &msg)
{
    collisionMsg = *msg;
}

void MissionPlanning::navCallback_(const messages::NavFilterOut::ConstPtr &msg)
{
    robotStatus.rollAngle = msg->roll;
    robotStatus.pitchAngle = msg->pitch;
    navStatus = msg->nav_status;
    navStopRequest = msg->stop_request;
    nb1Good = msg->nb1_good;
    nb2Good = msg->nb2_good;
    if(msg->pitch > biasRemovalTiltLimit) tiltTooExtremeForBiasRemoval = true;
    else tiltTooExtremeForBiasRemoval = false;
}

void MissionPlanning::execInfoCallback_(const messages::ExecInfo::ConstPtr &msg)
{
    execInfoMsg = *msg;
}

void MissionPlanning::lidarFilterCallback_(const messages::LidarFilterOut::ConstPtr &msg)
{
    lidarFilterMsg = *msg;
}

void MissionPlanning::hsmMasterStatusCallback_(const messages::MasterStatus::ConstPtr &msg)
{
    if(msg->navSolutionsDiverged) performSafeMode = true;
    else performSafeMode = false;
    if(!msg->navSolutionsDiverged && hsmMasterStatusMsg.navSolutionsDiverged) avoid.sendWait(shortRecomputeActionWaitTime, true);
    hsmMasterStatusMsg = *msg;
}

void MissionPlanning::cvSamplesCallback_(const messages::CVSamplesFound::ConstPtr &msg)
{
    cvSamplesFoundMsg = *msg;
    ROS_INFO("+++cvSamplesCallback_+++");
    ROS_INFO("sampleList size = %u",cvSamplesFoundMsg.sampleList.size());
    if(cvSamplesFoundMsg.sampleList.size()>0)
    {
        for(int i=0; i<cvSamplesFoundMsg.sampleList.size(); i++)
        {
            ROS_INFO("\nsampleList[%i]",i);
            ROS_INFO("conf = %f",cvSamplesFoundMsg.sampleList.at(i).confidence);
            ROS_INFO("distance = %f",cvSamplesFoundMsg.sampleList.at(i).distance);
            ROS_INFO("bearing = %f",cvSamplesFoundMsg.sampleList.at(i).bearing);
            ROS_INFO("white = %i",cvSamplesFoundMsg.sampleList.at(i).white);
            ROS_INFO("silver = %i",cvSamplesFoundMsg.sampleList.at(i).silver);
            ROS_INFO("blueOrPurple = %i",cvSamplesFoundMsg.sampleList.at(i).blueOrPurple);
            ROS_INFO("pink = %i",cvSamplesFoundMsg.sampleList.at(i).pink);
            ROS_INFO("red = %i",cvSamplesFoundMsg.sampleList.at(i).red);
            ROS_INFO("oragne = %i",cvSamplesFoundMsg.sampleList.at(i).orange);
            ROS_INFO("yellow = %i",cvSamplesFoundMsg.sampleList.at(i).yellow);
        }
    }
    initialize.findHighestConfSample();
    if(sampleHistoryActive) initialize.sampleHistoryNewData(highestConfSample.confidence, highestConfSample.types);
    else initialize.startSampleHistory(highestConfSample.confidence, highestConfSample.types);
    if(sampleHistoryActive) initialize.sampleHistoryComputeModifiedConf();
    sampleDataActedUpon = false;
}

void MissionPlanning::nextWaypointCallback_(const messages::NextWaypointOut::ConstPtr &msg)
{
    nextWaypointMsg = *msg;
}

void MissionPlanning::grabberStatusCallback_(const messages::ExecGrabberStatus::ConstPtr &msg)
{
    grabberStatusMsg = *msg;
}

bool MissionPlanning::emergencyEscapeCallback_(messages::EmergencyEscapeTrigger::Request &req, messages::EmergencyEscapeTrigger::Response &res)
{
    escapeCondition = req.escapeCondition;
    return true;
}

bool MissionPlanning::controlCallback_(messages::MissionPlanningControl::Request &req, messages::MissionPlanningControl::Response &res)
{
    initialized = req.initialized;
    escapeCondition = req.escapeCondition;
    escapeLockout = req.escapeLockout;
    avoidLockout = req.avoidLockout;
    shouldExecuteAvoidManeuver = req.shouldExecuteAvoidManeuver;
    performBiasRemoval = req.performBiasRemoval;
    performHoming = req.performHoming;
    inSearchableRegion = req.inSearchableRegion;
    roiTimeExpired = req.roiTimeExpired;
    possessingSample = req.possessingSample;
    possibleSample = req.possibleSample;
    definiteSample = req.definiteSample;
    sampleInCollectPosition = req.sampleInCollectPosition;
    sideOffsetGrab = req.sideOffsetGrab;
    performReorient = req.performReorient;
    confirmedPossession = req.confirmedPossession;
    atHome = req.atHome;
    homingUpdateFailed = req.homingUpdateFailed;
    performSafeMode = req.performSafeMode;
    inDepositPosition = req.inDepositPosition;
    missionEnded = req.missionEnded;
    useDeadReckoning = req.useDeadReckoning;
    possiblyLost = req.possiblyLost;
    roiKeyframed = req.roiKeyframed;
    startSLAM = req.startSLAM;
    giveUpROI = req.giveUpROI;
    robotStatus.pauseSwitch = req.pause;
    samplesCollected = req.samplesCollected;
    avoidCount = req.avoidCount;
    examineCount = req.examineCount;
    backUpCount = req.backupCount;
    confirmCollectFailedCount = req.confirmCollectFailedCount;
    reorientCount = req.reorientCount;
    homingUpdatedFailedCount = req.homingUpdatedFailedCount;
    for(int i=0; i<req.numProcs; i++)
    {
        procsToExecute[i] = req.procsToExecute.at(i);
        procsToInterrupt[i] = req.procsToInterrupt.at(i);
        procsBeingExecuted[i] = req.procsBeingExecuted.at(i);
        procsToResume[i] = req.procsToResume.at(i);
        switch(static_cast<PROC_TYPES_T>(i)) // If PROC_TYPES_T enum is ever edited, edit this as well
        {
        case __initialize__:
            initialize.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __emergencyEscape__:
            emergencyEscape.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __avoid__:
            avoid.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __biasRemoval__:
            biasRemoval.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __nextBestRegion__:
            nextBestRegion.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __searchRegion__:
            searchRegion.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __examine__:
            examine.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __approach__:
            approach.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __collect__:
            collect.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __confirmCollect__:
            confirmCollect.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __reorient__:
            reorient.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __goHome__:
            goHome.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __squareUpdate__:
            squareUpdate.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __depositApproach__:
            depositApproach.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __depositSample__:
            depositSample.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __safeMode__:
            safeMode.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __sosMode__:
            sosMode.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        }
    }
    collisionMsg.collision = req.collisionCondition;
    missionTime = req.missionTime;
    currentROIIndex = req.currentROIIndex;
    return true;
}

void MissionPlanning::roiTimeExpiredCallback_(const ros::TimerEvent &event)
{
    roiTimeExpired = true;
    ROS_WARN("roiTimeExpiredCallback");
    voiceSay->call("r o i time expired");
}

void MissionPlanning::biasRemovalTimerCallback_(const ros::TimerEvent &event)
{
    performBiasRemoval = true;
    ROS_INFO("biasRemovalTimer expired");
    voiceSay->call("bias removal timer expired");
}

void MissionPlanning::homingTimerCallback_(const ros::TimerEvent &event)
{
    performHoming = true;
    ROS_INFO("homingTimerExpired");
    voiceSay->call("homing timer expired");
}

void MissionPlanning::searchTimerCallback_(const ros::TimerEvent &event)
{
    searchTimedOut = true;
    ROS_INFO("searchTimer expired");
    voiceSay->call("search timer expired");
}

void MissionPlanning::biasRemovalActionTimerCallback_(const ros::TimerEvent &event)
{
    ROS_WARN("biasRemovalTimedOut");
    biasRemovalTimedOut = true;
}

void MissionPlanning::queueEmptyTimerCallback_(const ros::TimerEvent &event)
{
    queueEmptyTimedOut = true;
    ROS_WARN("queue empty timer expired");
    voiceSay->call("queue empty timer expired");
}

void MissionPlanning::roiOvertimeTimerCallback_(const ros::TimerEvent &event)
{
    roiOvertimeExpired = true;
    ROS_WARN("roi overtime expired");
    voiceSay->call("r o i overtime expired");
}
