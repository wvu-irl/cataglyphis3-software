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
#include <armadillo>
#include <math.h>

#define NUM_PROC_TYPES 8
enum PROC_TYPES_T {avoid__, returnHome__, deposit__, acquire__, examine__, planRegionPath__, chooseRegion__, init__};

class MissionPlanningProcessShare
{
public:
    static std::vector<bool> procsToExecute;
    static std::vector<bool> procsBeingExecuted;
    static ros::ServiceClient woiClient;
    static robot_control::WaypointsOfInterest woiSrv;
    static ros::ServiceClient execActionClient;
    static messages::ExecAction execActionSrv;
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
};

std::vector<bool> MissionPlanningProcessShare::procsToExecute;
std::vector<bool> MissionPlanningProcessShare::procsBeingExecuted;
ros::ServiceClient MissionPlanningProcessShare::woiClient;
robot_control::WaypointsOfInterest MissionPlanningProcessShare::woiSrv;
ros::ServiceClient MissionPlanningProcessShare::execActionClient;
messages::ExecAction MissionPlanningProcessShare::execActionSrv;
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

#endif // MISSION_PLANNING_PROCESS_SHARE_H
