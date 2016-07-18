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
    globalMapPathHazardsClient = nh.serviceClient<messages::GlobalMapPathHazards>("/control/mapmanager/globalmappathhazards");
    nb1Sub = nh.subscribe<messages::nb1_to_i7_msg>("hw_interface/nb1in/nb1in", 1, &MissionPlanning::nb1Callback_, this);
    collisionSub = nh.subscribe<messages::CollisionOut>("lidar/collisiondetectionout/collisiondetectionout", 1, &MissionPlanning::collisionCallback_, this);
    execInfoSub = nh.subscribe<messages::ExecInfo>("control/exec/info", 1, &MissionPlanning::execInfoCallback_, this);
    lidarFilterSub = nh.subscribe<messages::LidarFilterOut>("lidar/lidarfilteringout/lidarfilteringout", 1, &MissionPlanning::lidarFilterCallback_, this);
    hsmMasterStatusSub = nh.subscribe<messages::MasterStatus>("hsm/masterexecutive/masterstatus", 1, &MissionPlanning::hsmMasterStatusCallback_, this);
    cvSamplesSub = nh.subscribe<messages::CVSamplesFound>("vision/samplesearch/samplesearchout", 1, &MissionPlanning::cvSamplesCallback_, this);
    emergencyEscapeServ = nh.advertiseService("/control/missionplanning/emergencyescapetrigger", &MissionPlanning::emergencyEscapeCallback_, this);
    controlServ = nh.advertiseService("/control/missionplanning/control", &MissionPlanning::controlCallback_, this);
    infoPub = nh.advertise<messages::MissionPlanningInfo>("/control/missionplanning/info", 1);
    driveSpeedsPub = nh.advertise<robot_control::DriveSpeeds>("/control/missionplanning/drivespeeds", 1);
    collisionInterruptTrigger = false;
    escapeCondition = false;
    inSearchableRegion = false;
    roiTimeExpired = false;
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
    avoidLockout = true;
    escapeLockout = false;
    roiKeyframed = false;
    avoidCount = 0;
    prevAvoidCountDecXPos = robotStatus.xPos;
    prevAvoidCountDecYPos = robotStatus.yPos;
    execLastProcType = __depositSample__;
    execLastSerialNum = 99;
    allocatedROITime = 480.0; // sec == 8 min
    collisionInterruptThresh = 1.0; // m
    emergencyEscape.reg(__emergencyEscape__);
    avoid.reg(__avoid__);
    nextBestRegion.reg(__nextBestRegion__); // consider polymorphic constructor
    searchRegion.reg(__searchRegion__);
    examine.reg(__examine__);
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
    examineCount = 0;
    backUpCount = 0;
    confirmCollectFailedCount = 0;
    driveSpeedsMsg.vMax = defaultVMax;
    driveSpeedsMsg.rMax = defaultRMax;
    driveSpeedsMsgPrev.vMax = 0.0;
    driveSpeedsMsgPrev.rMax = 0.0;
    driveSpeedsPub.publish(driveSpeedsMsg);
    infoMsg.procsToExecute.resize(NUM_PROC_TYPES,0);
    infoMsg.procsToInterrupt.resize(NUM_PROC_TYPES,0);
    infoMsg.procsBeingExecuted.resize(NUM_PROC_TYPES,0);
    srand(time(NULL));
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

    ROS_INFO_THROTTLE(3,"Mission Planning running...");
    evalConditions_();
    //ROS_DEBUG("after evalConditions");
    ROS_DEBUG("robotStatus.pauseSwitch = %i",robotStatus.pauseSwitch);
    if(robotStatus.pauseSwitch) runPause_();
    else runProcesses_();
    packAndPubInfoMsg_();
    //std::printf("\n");
}

void MissionPlanning::evalConditions_()
{
    int temp;
    if(multiProcLockout)
    {
        multiProcLockout = true;
        robotStatus.pauseSwitch = true;
        ROS_FATAL_THROTTLE(3,"tried to execute multiple procedures..........");
    }
    else
    {
        ROS_INFO("=========================================");
        ROS_INFO("escapeCondition = %i",escapeCondition);
        ROS_INFO("escapeLockout = %i",escapeLockout);
        ROS_INFO("collisionCondition = %i",collisionMsg.collision);
        ROS_INFO("avoidLockout = %i",avoidLockout);
        ROS_INFO("inSearchableRegion = %i",inSearchableRegion);
        ROS_INFO("roiTimeExpired = %i",roiTimeExpired);
        ROS_INFO("possessingSample = %i",possessingSample);
        ROS_INFO("possibleSample = %i",possibleSample);
        ROS_INFO("definiteSample = %i",definiteSample);
        //ROS_INFO("sampleDataActedUpon = %i",sampleDataActedUpon);
        ROS_INFO("sampleInCollectPosition = %i",sampleInCollectPosition);
        ROS_INFO("confirmedPossession = %i",confirmedPossession);
        ROS_INFO("atHome = %i",atHome);
        ROS_INFO("inDepositPosition = %i",inDepositPosition);
        ROS_INFO("avoidCount = %u",avoidCount);
        ROS_INFO("turnFlag = %i",execInfoMsg.turnFlag);
        ROS_INFO("stopFlag = %i",execInfoMsg.stopFlag);
        ROS_INFO("execDequeSize = %u",execInfoMsg.actionDequeSize);
        std::printf("actionDeque: (");
        for(int i=0; i<execInfoMsg.actionDequeSize; i++) std::printf("%i,",execInfoMsg.actionDeque[i]);
        std::printf(")\n");
        std::printf("actionFloat1: (");
        for(int i=0; i<execInfoMsg.actionDequeSize; i++) std::printf("%f,",execInfoMsg.actionFloat1[i]);
        std::printf(")\n");
        std::printf("actionFloat2: (");
        for(int i=0; i<execInfoMsg.actionDequeSize; i++) std::printf("%f,",execInfoMsg.actionFloat2[i]);
        std::printf(")\n");
        //for(int i; i<NUM_PROC_TYPES; i++) {procsToExecute.at(i) = false; procsToInterrupt.at(i) = false;}
        calcnumProcsBeingOrToBeExec_();
        if(escapeCondition && !execInfoMsg.stopFlag && !escapeLockout && !missionEnded) //  Emergency Escape
        {
            for(int i=0; i<NUM_PROC_TYPES; i++) procsToInterrupt[i] = procsBeingExecuted[i];
            if(procsBeingExecuted[__emergencyEscape__]) {procsToInterrupt[__emergencyEscape__] = true; ROS_INFO("to interrupt emergencyEscape");}
            else {procsToExecute[__emergencyEscape__] = true; procsToInterrupt[__emergencyEscape__] = false; ROS_INFO("to execute emergencyEscape");}
        }
        //calcnumProcsBeingOrToBeExec_();
        if(!escapeCondition && collisionMsg.collision!=0 && !execInfoMsg.turnFlag && !execInfoMsg.stopFlag && !avoidLockout && !missionEnded) // Avoid
        {
            shouldExecuteAvoidManeuver = true;
            for(int i=0; i<NUM_PROC_TYPES; i++) procsToInterrupt[i] = procsBeingExecuted[i];
            procsToInterrupt[__avoid__] = false;
            if(procsToInterrupt[__emergencyEscape__]) avoid.dequeClearFront = true; // If avoid occured during emergency escape, just treat the avoid maneuver as the offset drive in emergency escape
            if(procsToInterrupt[__nextBestRegion__] || procsToInterrupt[__searchRegion__] || procsToInterrupt[__goHome__]) // If avoid occured while driving to a waypoint globally (which occurs in these procedures), check if the remaining distance to the waypoint is small enough to just end the drive there
            {
                ROS_INFO("was executing proc with driveGlobal");
                if(static_cast<ACTION_TYPE_T>(execInfoMsg.actionDeque[0]) == _driveGlobal)
                {
                    ROS_INFO("was executing driveGlobal");
                    avoidRemainingWaypointDistance = hypot(execInfoMsg.actionFloat1[0] - robotStatus.xPos, execInfoMsg.actionFloat2[0] - robotStatus.yPos);
                    ROS_INFO("avoidRemainingWaypointDistance = %f",avoidRemainingWaypointDistance);
                    if(avoidRemainingWaypointDistance <= minAvoidRemainingWaypointDistance)
                    {
                        ROS_INFO("closer than avoid remaining waypoint distance. End drive here");
                        avoid.sendDequeClearFront();
                        shouldExecuteAvoidManeuver = false;
                        avoidLockout = true;
                    }
                }
                /*robotStatus.pauseSwitch = true;
                pause.sendPause();
                std::cout << "press enter to continue" << std::endl;
                std::cin >> temp;
                robotStatus.pauseSwitch = false;
                pause.sendUnPause();*/
            }
            if(shouldExecuteAvoidManeuver)
            {
                if(!procsBeingExecuted[__avoid__]) {procsToExecute[__avoid__] = true; ROS_INFO("to execute avoid");}
                else if((collisionMsg.distance_to_collision <= collisionInterruptThresh) && procsBeingExecuted[__avoid__]) {procsToInterrupt[__avoid__] = true; ROS_INFO("to interrput avoid");}
            }
        }
        //calcnumProcsBeingOrToBeExec_();
        if(numProcsBeingOrToBeExec==0 && !possessingSample && !confirmedPossession && !(possibleSample || definiteSample) && !inSearchableRegion && !escapeCondition && !missionEnded) // Next Best Region
        {
            procsToExecute[__nextBestRegion__] = true;
            ROS_INFO("to execute nextBestRegion");
            /*robotStatus.pauseSwitch = true;
            pause.sendPause();
            std::cout << "press enter to continue" << std::endl;
            std::cin >> temp;
            robotStatus.pauseSwitch = false;
            pause.sendUnPause();*/
        }
        //calcnumProcsBeingOrToBeExec_();
        if(numProcsBeingOrToBeExec==0 && !possessingSample && !confirmedPossession && !(possibleSample || definiteSample) && inSearchableRegion && !escapeCondition && !missionEnded) // Search Region
        {
            procsToExecute[__searchRegion__] = true;
            ROS_INFO("to execute searchRegion");
            /*robotStatus.pauseSwitch = true;
            pause.sendPause();
            std::cout << "press enter to continue" << std::endl;
            std::cin >> temp;
            robotStatus.pauseSwitch = false;
            pause.sendUnPause();*/
        }
        //calcnumProcsBeingOrToBeExec_();
        if(numProcsBeingOrToBeExec==0 && !possessingSample && !confirmedPossession && possibleSample && !definiteSample && !escapeCondition && !missionEnded) // Examine
        {
            procsToExecute[__examine__] = true;
            ROS_INFO("to execute examine");
        }
        //calcnumProcsBeingOrToBeExec_();
        if(numProcsBeingOrToBeExec==0 && !possessingSample && !confirmedPossession && definiteSample && !sampleInCollectPosition && !escapeCondition && !missionEnded) // Approach
        {
            procsToExecute[__approach__] = true;
            ROS_INFO("to execute approach");
        }
        //calcnumProcsBeingOrToBeExec_();
        if(numProcsBeingOrToBeExec==0 && sampleInCollectPosition && !possessingSample && !confirmedPossession && !escapeCondition && !missionEnded) // Collect
        {
            procsToExecute[__collect__] = true;
            ROS_INFO("to execute collect");
        }
        //calcnumProcsBeingOrToBeExec_();
        if(numProcsBeingOrToBeExec==0 && possessingSample && !confirmedPossession && !escapeCondition && !missionEnded) // Confirm Collect
        {
            procsToExecute[__confirmCollect__] = true;
            ROS_INFO("to execute confirmCollect");
        }
        //calcnumProcsBeingOrToBeExec_();
        if(numProcsBeingOrToBeExec==0 && possessingSample && confirmedPossession && !atHome && !escapeCondition && !missionEnded) // Go Home
        {
            procsToExecute[__goHome__] = true;
            ROS_INFO("to execute goHome");
        }
        //calcnumProcsBeingOrToBeExec_();
        if(numProcsBeingOrToBeExec==0 && possessingSample && confirmedPossession && atHome && !inDepositPosition && !escapeCondition && !missionEnded) // Deposit Approach
        {
            procsToExecute[__depositApproach__] = true;
            ROS_INFO("to execute depositApproach");
        }
        //calcnumProcsBeingOrToBeExec_();
        if(numProcsBeingOrToBeExec==0 && possessingSample && confirmedPossession && atHome && inDepositPosition && !escapeCondition && !missionEnded) // Deposit Sample
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
    emergencyEscape.run();
    avoid.run();
    nextBestRegion.run();
    searchRegion.run();
    examine.run();
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

void MissionPlanning::calcnumProcsBeingOrToBeExec_()
{
    numProcsBeingOrToBeExec = 0;
    for(int i=0; i<NUM_PROC_TYPES; i++) 
    {
        if(procsBeingExecuted[i] || procsToExecute[i]) numProcsBeingOrToBeExec++;
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

void MissionPlanning::packAndPubInfoMsg_()
{
    infoMsg.pause = robotStatus.pauseSwitch;
    infoMsg.escapeCondition = escapeCondition;
    infoMsg.escapeLockout = escapeLockout;
    infoMsg.collisionCondition = collisionMsg.collision;
    infoMsg.avoidLockout = avoidLockout;
    infoMsg.inSearchableRegion = inSearchableRegion;
    infoMsg.roiTimeExpired = roiTimeExpired;
    infoMsg.possessingSample = possessingSample;
    infoMsg.possibleSample = possibleSample;
    infoMsg.definiteSample = definiteSample;
    infoMsg.sampleInCollectPosition = sampleInCollectPosition;
    infoMsg.confirmedPossession = confirmedPossession;
    infoMsg.atHome = atHome;
    infoMsg.inDepositPosition = inDepositPosition;
    infoMsg.samplesCollected = samplesCollected;
    infoMsg.avoidCount = avoidCount;
    infoMsg.examineCount = examineCount;
    infoMsg.backupCount = backUpCount;
    infoMsg.confirmCollectFailedCount = confirmCollectFailedCount;
    infoMsg.roiKeyframed = roiKeyframed;
    infoMsg.stopFlag = execInfoMsg.stopFlag;
    infoMsg.turnFlag = execInfoMsg.turnFlag;
    for(int i=0; i<NUM_PROC_TYPES; i++)
    {
        infoMsg.procsToExecute.at(i) = procsToExecute[i];
        infoMsg.procsToInterrupt.at(i) = procsToInterrupt[i];
        infoMsg.procsBeingExecuted.at(i) = procsBeingExecuted[i];
    }
    infoMsg.missionEnded = missionEnded;
    infoPub.publish(infoMsg);
}

void MissionPlanning::poseCallback_(const messages::RobotPose::ConstPtr& msg)
{
    robotStatus.xPos = msg->x;
    robotStatus.yPos = msg->y;
	robotStatus.heading = msg->heading;
    robotStatus.bearing = RAD2DEG*atan2(msg->y, msg->x);
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

void MissionPlanning::lidarFilterCallback_(const messages::LidarFilterOut::ConstPtr &msg)
{
    lidarFilterMsg = *msg;
}

void MissionPlanning::hsmMasterStatusCallback_(const messages::MasterStatus::ConstPtr &msg)
{
    hsmMasterStatusMsg = *msg;
}

void MissionPlanning::cvSamplesCallback_(const messages::CVSamplesFound::ConstPtr &msg)
{
    cvSamplesFoundMsg = *msg;
    updateSampleFlags_();
    sampleDataActedUpon = false;
}

bool MissionPlanning::emergencyEscapeCallback_(messages::EmergencyEscapeTrigger::Request &req, messages::EmergencyEscapeTrigger::Response &res)
{
    escapeCondition = req.escapeCondition;
    return true;
}

bool MissionPlanning::controlCallback_(messages::MissionPlanningControl::Request &req, messages::MissionPlanningControl::Response &res)
{
    escapeCondition = req.escapeCondition;
    escapeLockout = req.escapeLockout;
    inSearchableRegion = req.inSearchableRegion;
    roiTimeExpired = req.roiTimeExpired;
    possessingSample = req.possessingSample;
    possibleSample = req.possibleSample;
    definiteSample = req.definiteSample;
    sampleInCollectPosition = req.sampleInCollectPosition;
    confirmedPossession = req.confirmedPossession;
    atHome = req.atHome;
    inDepositPosition = req.inDepositPosition;
    samplesCollected = req.samplesCollected;
    avoidCount = req.avoidCount;
    examineCount = req.examineCount;
    backUpCount = req.backupCount;
    confirmCollectFailedCount = req.confirmCollectFailedCount;
    roiKeyframed = req.roiKeyframed;
    for(int i=0; i<req.numProcs; i++)
    {
        procsToExecute[i] = req.procsToExecute.at(i);
        procsToInterrupt[i] = req.procsToInterrupt.at(i);
        procsBeingExecuted[i] = req.procsBeingExecuted.at(i);
    }
    missionEnded = req.missionEnded;
    if(req.setProcState)
    {
        switch(static_cast<PROC_TYPES_T>(req.setProcStateIndex)) // If PROC_TYPES_T enum is ever edited, edit this as well
        {
        case __emergencyEscape__:
            emergencyEscape.state = static_cast<PROC_STATE_T>(req.setProcStateValue);
            break;
        case __avoid__:
            avoid.state = static_cast<PROC_STATE_T>(req.setProcStateValue);
            break;
        case __nextBestRegion__:
            nextBestRegion.state = static_cast<PROC_STATE_T>(req.setProcStateValue);
            break;
        case __searchRegion__:
            searchRegion.state = static_cast<PROC_STATE_T>(req.setProcStateValue);
            break;
        case __examine__:
            examine.state = static_cast<PROC_STATE_T>(req.setProcStateValue);
            break;
        case __approach__:
            approach.state = static_cast<PROC_STATE_T>(req.setProcStateValue);
            break;
        case __collect__:
            collect.state = static_cast<PROC_STATE_T>(req.setProcStateValue);
            break;
        case __confirmCollect__:
            confirmCollect.state = static_cast<PROC_STATE_T>(req.setProcStateValue);
            break;
        case __goHome__:
            goHome.state = static_cast<PROC_STATE_T>(req.setProcStateValue);
            break;
        case __depositApproach__:
            depositApproach.state = static_cast<PROC_STATE_T>(req.setProcStateValue);
            break;
        case __depositSample__:
            depositSample.state = static_cast<PROC_STATE_T>(req.setProcStateValue);
            break;
        case __pause__:
            pause.state = static_cast<PROC_STATE_T>(req.setProcStateValue);
            break;
        }
    }
    return true;
}
