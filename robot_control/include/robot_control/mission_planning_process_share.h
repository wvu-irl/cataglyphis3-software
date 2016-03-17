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
#include <armadillo>
#include <math.h>

#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI
#define NUM_PROC_TYPES 9
enum PROC_TYPES_T {avoid__, returnHome__, deposit__, acquire__, examine__, planRegionPath__, chooseRegion__, init__, pause__};

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

#endif // MISSION_PLANNING_PROCESS_SHARE_H
