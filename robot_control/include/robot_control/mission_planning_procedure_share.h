#ifndef MISSION_PLANNING_PROCESS_SHARE_H
#define MISSION_PLANNING_PROCESS_SHARE_H
#include <vector>
#include <ros/ros.h>
#include <robot_control/Waypoint.h>
#include <robot_control/RegionsOfInterest.h>
#include <robot_control/IntermediateWaypoints.h>
#include <robot_control/ModifyROI.h>
#include <robot_control/SearchMap.h>
#include <robot_control/RandomSearchWaypoints.h>
#include <robot_control/DriveSpeeds.h>
#include <robot_control/cataglyphis_timer.h>
#include <messages/MapPathHazards.h>
#include "robot_status.h"
#include "action_type_enum.h"
#include <messages/ExecAction.h>
#include <messages/ExecInfo.h>
#include <messages/CollisionOut.h>
#include <messages/CVSamplesFound.h>
#include <messages/LidarFilterOut.h>
#include <messages/MasterStatus.h>
#include <messages/NavFilterControl.h>
#include <messages/NextWaypointOut.h>
#include <messages/ExecGrabberStatus.h>
#include <hsm/voice.h>
#include <armadillo>
#include <math.h>
#include <time.h>
#include "mission_planning_types_defines.h"

class MissionPlanningProcedureShare
{
public:
	//static std::vector<bool> procsToExecute;
	//static std::vector<bool> procsToInterrupt;
	//static std::vector<bool> procsBeingExecuted;
	static bool procsToExecute[NUM_PROC_TYPES];
	static bool procsToInterrupt[NUM_PROC_TYPES];
	static bool procsBeingExecuted[NUM_PROC_TYPES];
	static bool procsToResume[NUM_PROC_TYPES];
	static unsigned int numProcsBeingOrToBeExecOrRes;
	static unsigned int numProcsBeingOrToBeExec;
	static unsigned int numProcsToBeExecAndNotInterrupt;
    static ros::ServiceClient execActionClient;
    static messages::ExecAction execActionSrv;
	static ros::Subscriber execInfoSub;
	static messages::ExecInfo execInfoMsg;
	static ros::Subscriber lidarFilterSub;
	static messages::LidarFilterOut lidarFilterMsg;
	static ros::Subscriber hsmMasterStatusSub;
	static messages::MasterStatus hsmMasterStatusMsg;
	static ros::Subscriber nextWaypointSub;
	static messages::NextWaypointOut nextWaypointMsg;
	static ros::Subscriber grabberStatusSub;
	static messages::ExecGrabberStatus grabberStatusMsg;
    static ros::ServiceClient intermediateWaypointsClient;
    static robot_control::IntermediateWaypoints intermediateWaypointsSrv;
    static ros::ServiceClient reqROIClient;
    static robot_control::RegionsOfInterest regionsOfInterestSrv;
	static ros::ServiceClient modROIClient;
	static robot_control::ModifyROI modROISrv;
	static ros::ServiceClient searchMapClient;
	static robot_control::SearchMap searchMapSrv;
	static ros::ServiceClient randomSearchWaypointsClient;
	static robot_control::RandomSearchWaypoints randomSearchWaypointsSrv;
	static ros::ServiceClient globalMapPathHazardsClient;
	static messages::MapPathHazards globalMapPathHazardsSrv;
	static ros::ServiceClient searchLocalMapPathHazardsClient;
	static messages::MapPathHazards searchLocalMapPathHazardsSrv;
	static ros::ServiceClient navControlClient;
	static messages::NavFilterControl navControlSrv;
	static VoiceBase* voiceSay;
	//#define voiceObj voiceSay;
	static ros::Publisher driveSpeedsPub;
	static robot_control::DriveSpeeds driveSpeedsMsg;
	static robot_control::DriveSpeeds driveSpeedsMsgPrev;
    static RobotStatus robotStatus;
	static CataglyphisTimerBase* timers[NUM_TIMERS];
    static std::vector<robot_control::Waypoint> waypointsToTravel;
    static int numWaypointsToTravel;
    static bool execDequeEmpty;
    static PROC_TYPES_T execLastProcType;
    static unsigned int execLastSerialNum;
	static messages::CollisionOut collisionMsg;
	static float collisionInterruptThresh; // m
	const float collisionMinDistance = 2.0; // m
	static ros::Subscriber cvSamplesSub;
	static messages::CVSamplesFound cvSamplesFoundMsg;
	const float distanceToGrabber = 0.86;
	const float distanceToBlindDriveLocation = 0.91; // m
	const float blindDriveDistance = 0.35; // m (0.39)
	const float sideGrabAngleOffset = 15.0; // deg
	const float initGrabberDistanceTolerance = 0.17; // m
	const float initGrabberAngleTolerance = 6.0; // deg
	const float grabberDistanceToleranceIncrementPerApproachManeuver = 0.01; // m
	const float grabberAngleToleranceIncrementPerApproachManeuver = 0.05; // deg
	const float possibleSampleConfThresh = 0.5; // Change corresponding value in map manager if ever changed
	const float definiteSampleConfThresh = 0.6;
	static int currentROIIndex;
	static int prevROIIndex;
	static bool initialized;
	static bool escapeCondition;
	static bool performBiasRemoval;
	static bool performHoming;
	static bool inSearchableRegion;
	static bool roiTimeExpired;
	static bool roiOvertimeExpired;
	static bool possessingSample;
	static bool possibleSample;
	static bool definiteSample;
	static bool sampleDataActedUpon;
	static bool sampleInCollectPosition;
	static bool sideOffsetGrab;
	static bool performReorient;
	static bool confirmedPossession;
	static bool atHome;
	static bool homingUpdateFailed;
	static bool performSafeMode;
	static bool inDepositPosition;
	static bool missionEnded;
	static bool useDeadReckoning;
	static bool possiblyLost;
	static unsigned int samplesCollected;
	static bool avoidLockout;
	static bool escapeLockout;
	static bool roiKeyframed;
	static bool startSLAM;
	static bool giveUpROI;
	static bool newSearchActionOnExec;
	static bool searchTimedOut;
	static bool tiltTooExtremeForBiasRemoval;
	static bool biasRemovalTimedOut;
	static bool navStopRequest;
	static bool queueEmptyTimedOut;
	static bool nb1Good;
	static bool nb2Good;
	static bool nb1Pause;
	static bool nb2Pause;
	static unsigned int avoidCount;
	const unsigned int maxNormalWaypointAvoidCount = 3;
	const unsigned int maxROIWaypointAvoidCount = 6;
	const unsigned int maxCornerWaypointAvoidCount = 12;
	const unsigned int maxHomeWaypointAvoidCount = 20;
	const unsigned int fullyBlockedAvoidCountIncrement = 5;
	const float minAvoidRemainingWaypointDistance = 4.0; // m
	const float metersPerAvoidCountDecrement = 8.0;
	static float prevAvoidCountDecXPos;
	static float prevAvoidCountDecYPos;
	static unsigned int numSampleCandidates;
	static std::vector<float> sampleValues;
	static float sampleDistanceAdjustedConf;
	static float distanceToDrive; // m
	static float angleToTurn; // deg
	static float expectedSampleDistance;
	static float expectedSampleAngle;
	//static messages::CVSampleProps highestConfSample;
	static CV_SAMPLE_PROPS_T highestConfSample;
	static bool sampleHistoryActive;
	static float sampleHistoryBestSampleConf;
	static float sampleHistoryModifiedConf;
	static bool sampleHistoryTypes[NUM_SAMPLE_TYPES];
	static int sampleHistoryGoodCount;
	static int sampleHistoryBadCount;
	const float sampleHistoryGoodGain = 0.05;
	const float sampleHistoryBadGain = 0.1;
	static float allocatedROITime; // sec
	static unsigned int examineCount;
	static unsigned int backUpCount;
	static unsigned int reorientCount;
	static unsigned int confirmCollectFailedCount;
	static unsigned int homingUpdatedFailedCount;
	static unsigned int navStatus;
	static double missionTime;
	static double prevTime;
	static bool missionStarted;
	const unsigned int maxHomingUpdatedFailedCount = 1;
	const unsigned int homingFailedSwitchToDeadReckoningCount = 1;
	const float sampleConfidenceGain = 1.0;
	const float sampleDistanceToExpectedGain = 0.3;
	const float defaultVMax = 1.2; // m/s
	const float fastVMax = 1.4; // m/s
	const float slowVMax = 0.72; // m/s
	const float defaultRMax = 45.0; // deg/s
	const float homeWaypointX = 5.0; // m
	const float homeWaypointY = 0.0; // m
	const float lidarUpdateWaitTime = 2.0; // sec
	const float biasRemovalTimeoutPeriod = 300.0; // sec = 5 minutes
	const float homingTimeoutPeriod = 840.0; // sec = 16 minutes
	const float searchTimeoutPeriod = 15.0; // sec
	const float sampleFoundNewROIProbMultiplier = 0.0;
	const float roiTimeExpiredNewSampleProbMultiplier = 0.05;
	const float giveUpROIFromAvoidNewSampleProbMultiplier = 0.01;
	const float biasRemovalTiltLimit = 5.0; // deg
	const float biasRemovalActionTimeoutTime = 20.0; // sec
	const float shortRecomputeActionWaitTime = 0.1; // sec
	const float queueEmptyTimerPeriod = 30.0; // sec
	const float roiOvertimePeriod = 120.0; // sec
};

//std::vector<bool> MissionPlanningProcedureShare::procsToExecute;
//std::vector<bool> MissionPlanningProcedureShare::procsToInterrupt;
//std::vector<bool> MissionPlanningProcedureShare::procsBeingExecuted;
bool MissionPlanningProcedureShare::procsToExecute[NUM_PROC_TYPES];
bool MissionPlanningProcedureShare::procsToInterrupt[NUM_PROC_TYPES];
bool MissionPlanningProcedureShare::procsBeingExecuted[NUM_PROC_TYPES];
bool MissionPlanningProcedureShare::procsToResume[NUM_PROC_TYPES];
unsigned int MissionPlanningProcedureShare::numProcsBeingOrToBeExecOrRes;
unsigned int MissionPlanningProcedureShare::numProcsBeingOrToBeExec;
unsigned int MissionPlanningProcedureShare::numProcsToBeExecAndNotInterrupt;
ros::ServiceClient MissionPlanningProcedureShare::execActionClient;
messages::ExecAction MissionPlanningProcedureShare::execActionSrv;
ros::Subscriber MissionPlanningProcedureShare::execInfoSub;
messages::ExecInfo MissionPlanningProcedureShare::execInfoMsg;
ros::Subscriber MissionPlanningProcedureShare::lidarFilterSub;
messages::LidarFilterOut MissionPlanningProcedureShare::lidarFilterMsg;
ros::Subscriber MissionPlanningProcedureShare::hsmMasterStatusSub;
messages::MasterStatus MissionPlanningProcedureShare::hsmMasterStatusMsg;
ros::Subscriber MissionPlanningProcedureShare::nextWaypointSub;
messages::NextWaypointOut MissionPlanningProcedureShare::nextWaypointMsg;
ros::Subscriber MissionPlanningProcedureShare::grabberStatusSub;
messages::ExecGrabberStatus MissionPlanningProcedureShare::grabberStatusMsg;
ros::ServiceClient MissionPlanningProcedureShare::intermediateWaypointsClient;
robot_control::IntermediateWaypoints MissionPlanningProcedureShare::intermediateWaypointsSrv;
ros::ServiceClient MissionPlanningProcedureShare::reqROIClient;
robot_control::RegionsOfInterest MissionPlanningProcedureShare::regionsOfInterestSrv;
ros::ServiceClient MissionPlanningProcedureShare::modROIClient;
robot_control::ModifyROI MissionPlanningProcedureShare::modROISrv;
ros::ServiceClient MissionPlanningProcedureShare::searchMapClient;
robot_control::SearchMap MissionPlanningProcedureShare::searchMapSrv;
ros::ServiceClient MissionPlanningProcedureShare::randomSearchWaypointsClient;
robot_control::RandomSearchWaypoints MissionPlanningProcedureShare::randomSearchWaypointsSrv;
ros::ServiceClient MissionPlanningProcedureShare::globalMapPathHazardsClient;
messages::MapPathHazards MissionPlanningProcedureShare::globalMapPathHazardsSrv;
ros::ServiceClient MissionPlanningProcedureShare::searchLocalMapPathHazardsClient;
messages::MapPathHazards MissionPlanningProcedureShare::searchLocalMapPathHazardsSrv;
VoiceBase* MissionPlanningProcedureShare::voiceSay;
ros::ServiceClient MissionPlanningProcedureShare::navControlClient;
messages::NavFilterControl MissionPlanningProcedureShare::navControlSrv;
ros::Publisher MissionPlanningProcedureShare::driveSpeedsPub;
robot_control::DriveSpeeds MissionPlanningProcedureShare::driveSpeedsMsg;
robot_control::DriveSpeeds MissionPlanningProcedureShare::driveSpeedsMsgPrev;
RobotStatus MissionPlanningProcedureShare::robotStatus;
CataglyphisTimerBase* MissionPlanningProcedureShare::timers[NUM_TIMERS];
std::vector<robot_control::Waypoint> MissionPlanningProcedureShare::waypointsToTravel;
int MissionPlanningProcedureShare::numWaypointsToTravel;
bool MissionPlanningProcedureShare::execDequeEmpty;
PROC_TYPES_T MissionPlanningProcedureShare::execLastProcType;
unsigned int MissionPlanningProcedureShare::execLastSerialNum;
messages::CollisionOut MissionPlanningProcedureShare::collisionMsg;
float MissionPlanningProcedureShare::collisionInterruptThresh;
ros::Subscriber MissionPlanningProcedureShare::cvSamplesSub;
messages::CVSamplesFound MissionPlanningProcedureShare::cvSamplesFoundMsg;
int MissionPlanningProcedureShare::currentROIIndex;
int MissionPlanningProcedureShare::prevROIIndex;
bool MissionPlanningProcedureShare::initialized;
bool MissionPlanningProcedureShare::escapeCondition;
bool MissionPlanningProcedureShare::performBiasRemoval;
bool MissionPlanningProcedureShare::performHoming;
bool MissionPlanningProcedureShare::inSearchableRegion;
bool MissionPlanningProcedureShare::roiTimeExpired;
bool MissionPlanningProcedureShare::roiOvertimeExpired;
bool MissionPlanningProcedureShare::possessingSample;
bool MissionPlanningProcedureShare::possibleSample;
bool MissionPlanningProcedureShare::definiteSample;
bool MissionPlanningProcedureShare::sampleDataActedUpon;
bool MissionPlanningProcedureShare::sampleInCollectPosition;
bool MissionPlanningProcedureShare::sideOffsetGrab;
bool MissionPlanningProcedureShare::performReorient;
bool MissionPlanningProcedureShare::confirmedPossession;
bool MissionPlanningProcedureShare::atHome;
bool MissionPlanningProcedureShare::homingUpdateFailed;
bool MissionPlanningProcedureShare::performSafeMode;
bool MissionPlanningProcedureShare::inDepositPosition;
bool MissionPlanningProcedureShare::missionEnded;
bool MissionPlanningProcedureShare::useDeadReckoning;
bool MissionPlanningProcedureShare::possiblyLost;
unsigned int MissionPlanningProcedureShare::samplesCollected;
bool MissionPlanningProcedureShare::avoidLockout;
bool MissionPlanningProcedureShare::escapeLockout;
bool MissionPlanningProcedureShare::roiKeyframed;
bool MissionPlanningProcedureShare::startSLAM;
bool MissionPlanningProcedureShare::giveUpROI;
bool MissionPlanningProcedureShare::tiltTooExtremeForBiasRemoval;
bool MissionPlanningProcedureShare::searchTimedOut;
bool MissionPlanningProcedureShare::newSearchActionOnExec;
bool MissionPlanningProcedureShare::biasRemovalTimedOut;
bool MissionPlanningProcedureShare::navStopRequest;
bool MissionPlanningProcedureShare::queueEmptyTimedOut;
bool MissionPlanningProcedureShare::nb1Good;
bool MissionPlanningProcedureShare::nb2Good;
bool MissionPlanningProcedureShare::nb1Pause;
bool MissionPlanningProcedureShare::nb2Pause;
unsigned int MissionPlanningProcedureShare::avoidCount;
float MissionPlanningProcedureShare::prevAvoidCountDecXPos;
float MissionPlanningProcedureShare::prevAvoidCountDecYPos;
unsigned int MissionPlanningProcedureShare::numSampleCandidates;
std::vector<float> MissionPlanningProcedureShare::sampleValues;
float MissionPlanningProcedureShare::sampleDistanceAdjustedConf;
float MissionPlanningProcedureShare::distanceToDrive;
float MissionPlanningProcedureShare::angleToTurn;
float MissionPlanningProcedureShare::expectedSampleDistance;
float MissionPlanningProcedureShare::expectedSampleAngle;
//messages::CVSampleProps MissionPlanningProcedureShare::highestConfSample;
CV_SAMPLE_PROPS_T MissionPlanningProcedureShare::highestConfSample;
bool MissionPlanningProcedureShare::sampleHistoryActive;
float MissionPlanningProcedureShare::sampleHistoryBestSampleConf;
float MissionPlanningProcedureShare::sampleHistoryModifiedConf;
bool MissionPlanningProcedureShare::sampleHistoryTypes[NUM_SAMPLE_TYPES];
int MissionPlanningProcedureShare::sampleHistoryGoodCount;
int MissionPlanningProcedureShare::sampleHistoryBadCount;
float MissionPlanningProcedureShare::allocatedROITime;
unsigned int MissionPlanningProcedureShare::examineCount;
unsigned int MissionPlanningProcedureShare::backUpCount;
unsigned int MissionPlanningProcedureShare::reorientCount;
unsigned int MissionPlanningProcedureShare::confirmCollectFailedCount;
unsigned int MissionPlanningProcedureShare::homingUpdatedFailedCount;
unsigned int MissionPlanningProcedureShare::navStatus;
double MissionPlanningProcedureShare::missionTime;
double MissionPlanningProcedureShare::prevTime;
bool MissionPlanningProcedureShare::missionStarted;

#endif // MISSION_PLANNING_PROCESS_SHARE_H
