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
#include <messages/GlobalMapPathHazards.h>
#include "robot_status.h"
#include "action_type_enum.h"
#include <messages/ExecAction.h>
#include <messages/ExecInfo.h>
#include <messages/CollisionOut.h>
#include <messages/CVSamplesFound.h>
#include <messages/LidarFilterOut.h>
#include <messages/MasterStatus.h>
#include <armadillo>
#include <math.h>
#include <time.h>
#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI
#define NUM_PROC_TYPES 11
#define MAX_SAMPLES 10
enum PROC_TYPES_T {__emergencyEscape__ ,__avoid__, __nextBestRegion__, __searchRegion__, __examine__, __approach__, __collect__, __confirmCollect__, __goHome__, __depositApproach__, __depositSample__, __pause__};
//enum PROC_TYPES_T {avoid__, returnHome__, deposit__, acquire__, examine__, planRegionPath__, chooseRegion__, init__, pause__};

class MissionPlanningProcessShare
{
public:
	//static std::vector<bool> procsToExecute;
	//static std::vector<bool> procsToInterrupt;
	//static std::vector<bool> procsBeingExecuted;
	static bool procsToExecute[NUM_PROC_TYPES];
	static bool procsToInterrupt[NUM_PROC_TYPES];
	static bool procsBeingExecuted[NUM_PROC_TYPES];
    static ros::ServiceClient execActionClient;
    static messages::ExecAction execActionSrv;
	static ros::Subscriber execInfoSub;
	static messages::ExecInfo execInfoMsg;
	static ros::Subscriber lidarFilterSub;
	static messages::LidarFilterOut lidarFilterMsg;
	static ros::Subscriber hsmMasterStatusSub;
	static messages::MasterStatus hsmMasterStatusMsg;
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
	static messages::GlobalMapPathHazards globalMapPathHazardsSrv;
	static ros::Publisher driveSpeedsPub;
	static robot_control::DriveSpeeds driveSpeedsMsg;
	static robot_control::DriveSpeeds driveSpeedsMsgPrev;
    static RobotStatus robotStatus;
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
	static messages::CVSampleProps bestSample;
	const float distanceToGrabber = 0.86; // m
	const float blindDriveDistance = 0.457; // m
	const float grabberDistanceTolerance = 0.15; // m
	const float grabberAngleTolerance = 6.0; // deg
	const float possibleSampleConfThresh = 0.7;
	const float definiteSampleConfThresh = 0.9;
	static int currentROIIndex;
	static bool escapeCondition;
	static bool inSearchableRegion;
	static bool possessingSample;
	static bool possibleSample;
	static bool definiteSample;
	static bool sampleDataActedUpon;
	static bool sampleInCollectPosition;
	static bool confirmedPossession;
	static bool atHome;
	static bool inDepositPosition;
	static bool missionEnded;
	static int samplesCollected;
	static bool avoidLockout;
	static bool escapeLockout;
	static bool roiKeyframed;
	static unsigned int avoidCount;
	const unsigned int maxAvoidCount = 5;
	const float metersPerAvoidCountDecrement = 5.0;
	static float prevAvoidCountDecXPos;
	static float prevAvoidCountDecYPos;
	static unsigned int numSampleCandidates;
	static std::vector<float> sampleValues;
	static float bestSampleValue;
	static float distanceToDrive; // m
	static float angleToTurn; // deg
	static float expectedSampleDistance;
	static float expectedSampleAngle;
	static messages::CVSampleProps highestConfSample;
	static float allocatedROITime; // sec
	static int examineCount;
	static int backUpCount;
	static int confirmCollectFailedCount;
	const float sampleConfidenceGain = 1.0;
	const float sampleDistanceToExpectedGain = 1.0;
	const float defaultVMax = 1.2; // m/s
	const float fastVMax = 1.4; // m/s
	const float slowVMax = 0.6; // m/s
	const float defaultRMax = 45.0; // deg/s
};

//std::vector<bool> MissionPlanningProcessShare::procsToExecute;
//std::vector<bool> MissionPlanningProcessShare::procsToInterrupt;
//std::vector<bool> MissionPlanningProcessShare::procsBeingExecuted;
bool MissionPlanningProcessShare::procsToExecute[NUM_PROC_TYPES];
bool MissionPlanningProcessShare::procsToInterrupt[NUM_PROC_TYPES];
bool MissionPlanningProcessShare::procsBeingExecuted[NUM_PROC_TYPES];
ros::ServiceClient MissionPlanningProcessShare::execActionClient;
messages::ExecAction MissionPlanningProcessShare::execActionSrv;
ros::Subscriber MissionPlanningProcessShare::execInfoSub;
messages::ExecInfo MissionPlanningProcessShare::execInfoMsg;
ros::Subscriber MissionPlanningProcessShare::lidarFilterSub;
messages::LidarFilterOut MissionPlanningProcessShare::lidarFilterMsg;
ros::Subscriber MissionPlanningProcessShare::hsmMasterStatusSub;
messages::MasterStatus MissionPlanningProcessShare::hsmMasterStatusMsg;
ros::ServiceClient MissionPlanningProcessShare::intermediateWaypointsClient;
robot_control::IntermediateWaypoints MissionPlanningProcessShare::intermediateWaypointsSrv;
ros::ServiceClient MissionPlanningProcessShare::reqROIClient;
robot_control::RegionsOfInterest MissionPlanningProcessShare::regionsOfInterestSrv;
ros::ServiceClient MissionPlanningProcessShare::modROIClient;
robot_control::ModifyROI MissionPlanningProcessShare::modROISrv;
ros::ServiceClient MissionPlanningProcessShare::searchMapClient;
robot_control::SearchMap MissionPlanningProcessShare::searchMapSrv;
ros::ServiceClient MissionPlanningProcessShare::randomSearchWaypointsClient;
robot_control::RandomSearchWaypoints MissionPlanningProcessShare::randomSearchWaypointsSrv;
ros::ServiceClient MissionPlanningProcessShare::globalMapPathHazardsClient;
messages::GlobalMapPathHazards MissionPlanningProcessShare::globalMapPathHazardsSrv;
ros::Publisher MissionPlanningProcessShare::driveSpeedsPub;
robot_control::DriveSpeeds MissionPlanningProcessShare::driveSpeedsMsg;
robot_control::DriveSpeeds MissionPlanningProcessShare::driveSpeedsMsgPrev;
RobotStatus MissionPlanningProcessShare::robotStatus;
std::vector<robot_control::Waypoint> MissionPlanningProcessShare::waypointsToTravel;
int MissionPlanningProcessShare::numWaypointsToTravel;
bool MissionPlanningProcessShare::execDequeEmpty;
PROC_TYPES_T MissionPlanningProcessShare::execLastProcType;
unsigned int MissionPlanningProcessShare::execLastSerialNum;
messages::CollisionOut MissionPlanningProcessShare::collisionMsg;
float MissionPlanningProcessShare::collisionInterruptThresh;
ros::Subscriber MissionPlanningProcessShare::cvSamplesSub;
messages::CVSamplesFound MissionPlanningProcessShare::cvSamplesFoundMsg;
messages::CVSampleProps MissionPlanningProcessShare::bestSample;
int MissionPlanningProcessShare::currentROIIndex;
bool MissionPlanningProcessShare::escapeCondition;
bool MissionPlanningProcessShare::inSearchableRegion;
bool MissionPlanningProcessShare::possessingSample;
bool MissionPlanningProcessShare::possibleSample;
bool MissionPlanningProcessShare::definiteSample;
bool MissionPlanningProcessShare::sampleDataActedUpon;
bool MissionPlanningProcessShare::sampleInCollectPosition;
bool MissionPlanningProcessShare::confirmedPossession;
bool MissionPlanningProcessShare::atHome;
bool MissionPlanningProcessShare::inDepositPosition;
bool MissionPlanningProcessShare::missionEnded;
int MissionPlanningProcessShare::samplesCollected;
bool MissionPlanningProcessShare::avoidLockout;
bool MissionPlanningProcessShare::escapeLockout;
bool MissionPlanningProcessShare::roiKeyframed;
unsigned int MissionPlanningProcessShare::avoidCount;
float MissionPlanningProcessShare::prevAvoidCountDecXPos;
float MissionPlanningProcessShare::prevAvoidCountDecYPos;
unsigned int MissionPlanningProcessShare::numSampleCandidates;
std::vector<float> MissionPlanningProcessShare::sampleValues;
float MissionPlanningProcessShare::bestSampleValue;
float MissionPlanningProcessShare::distanceToDrive;
float MissionPlanningProcessShare::angleToTurn;
float MissionPlanningProcessShare::expectedSampleDistance;
float MissionPlanningProcessShare::expectedSampleAngle;
messages::CVSampleProps MissionPlanningProcessShare::highestConfSample;
float MissionPlanningProcessShare::allocatedROITime;
int MissionPlanningProcessShare::examineCount;
int MissionPlanningProcessShare::backUpCount;
int MissionPlanningProcessShare::confirmCollectFailedCount;

#endif // MISSION_PLANNING_PROCESS_SHARE_H
