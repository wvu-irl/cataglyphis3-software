#ifndef MISSION_PLANNING_PROCESS_SHARE_H
#define MISSION_PLANNING_PROCESS_SHARE_H
#include <vector>
#include <ros/ros.h>
#include <robot_control/Waypoint.h>
#include <robot_control/WaypointsOfInterest.h> // Remove
#include <robot_control/RegionsOfInterest.h>
#include <robot_control/WaypointsOfRegion.h>
#include <robot_control/IntermediateWaypoints.h>
#include <robot_control/ModifyROI.h>
#include "robot_status.h"
#include "action_type_enum.h"
#include <messages/ExecAction.h>
#include <messages/ExecInfo.h>
#include <messages/CollisionOut.h>
#include <messages/CVSamplesFound.h>
#include <armadillo>
#include <math.h>

#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI
#define NUM_PROC_TYPES 11
#define MAX_SAMPLES 10
enum PROC_TYPES_T {__avoid__, __nextBestRegion__, __searchClosestRegion__, __examine__, __approach__, __collect__, __confirmCollect__, __goHome__, __depositApproach__, __depositSample__, __pause__};
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
    static ros::ServiceClient woiClient;
    static robot_control::WaypointsOfInterest woiSrv;
    static ros::ServiceClient execActionClient;
    static messages::ExecAction execActionSrv;
	static ros::Subscriber execInfoSub;
	static messages::ExecInfo execInfoMsg;
    static ros::ServiceClient intermediateWaypointsClient;
    static robot_control::IntermediateWaypoints intermediateWaypointsSrv;
    static ros::ServiceClient reqROIClient;
    static robot_control::RegionsOfInterest regionsOfInterestSrv;
    static RobotStatus robotStatus;
    static std::vector<robot_control::Waypoint> waypointsToTravel;
    static int numWaypointsToTravel;
    static bool execDequeEmpty;
    static PROC_TYPES_T execLastProcType;
    static unsigned int execLastSerialNum;
    static ros::ServiceClient modROIClient;
    static robot_control::ModifyROI modROISrv;
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
	const int possibleSampleConfThresh = 900;
	const int definiteSampleConfThresh = 900;
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
	static unsigned int numSampleCandidates;
	static std::vector<int> sampleValues;
	static int bestSampleValue;
	static float distanceToDrive; // m
	static float angleToTurn; // deg
	static float expectedSampleDistance;
	static float expectedSampleAngle;
	static messages::CVSampleProps highestConfSample;
	const int sampleConfidenceGain = 1000;
	const int sampleDistanceToExpectedGain = 1000000;
};

//std::vector<bool> MissionPlanningProcessShare::procsToExecute;
//std::vector<bool> MissionPlanningProcessShare::procsToInterrupt;
//std::vector<bool> MissionPlanningProcessShare::procsBeingExecuted;
bool MissionPlanningProcessShare::procsToExecute[NUM_PROC_TYPES];
bool MissionPlanningProcessShare::procsToInterrupt[NUM_PROC_TYPES];
bool MissionPlanningProcessShare::procsBeingExecuted[NUM_PROC_TYPES];
ros::ServiceClient MissionPlanningProcessShare::woiClient;
robot_control::WaypointsOfInterest MissionPlanningProcessShare::woiSrv;
ros::ServiceClient MissionPlanningProcessShare::execActionClient;
messages::ExecAction MissionPlanningProcessShare::execActionSrv;
ros::Subscriber MissionPlanningProcessShare::execInfoSub;
messages::ExecInfo MissionPlanningProcessShare::execInfoMsg;
ros::ServiceClient MissionPlanningProcessShare::intermediateWaypointsClient;
robot_control::IntermediateWaypoints MissionPlanningProcessShare::intermediateWaypointsSrv;
ros::ServiceClient MissionPlanningProcessShare::reqROIClient;
robot_control::RegionsOfInterest MissionPlanningProcessShare::regionsOfInterestSrv;
RobotStatus MissionPlanningProcessShare::robotStatus;
std::vector<robot_control::Waypoint> MissionPlanningProcessShare::waypointsToTravel;
int MissionPlanningProcessShare::numWaypointsToTravel;
bool MissionPlanningProcessShare::execDequeEmpty;
PROC_TYPES_T MissionPlanningProcessShare::execLastProcType;
unsigned int MissionPlanningProcessShare::execLastSerialNum;
ros::ServiceClient MissionPlanningProcessShare::modROIClient;
robot_control::ModifyROI MissionPlanningProcessShare::modROISrv;
messages::CollisionOut MissionPlanningProcessShare::collisionMsg;
float MissionPlanningProcessShare::collisionInterruptThresh;
ros::Subscriber MissionPlanningProcessShare::cvSamplesSub;
messages::CVSamplesFound MissionPlanningProcessShare::cvSamplesFoundMsg;
messages::CVSampleProps MissionPlanningProcessShare::bestSample;
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
unsigned int MissionPlanningProcessShare::numSampleCandidates;
std::vector<int> MissionPlanningProcessShare::sampleValues;
int MissionPlanningProcessShare::bestSampleValue;
float MissionPlanningProcessShare::distanceToDrive;
float MissionPlanningProcessShare::angleToTurn;
float MissionPlanningProcessShare::expectedSampleDistance;
float MissionPlanningProcessShare::expectedSampleAngle;
messages::CVSampleProps MissionPlanningProcessShare::highestConfSample;

#endif // MISSION_PLANNING_PROCESS_SHARE_H
