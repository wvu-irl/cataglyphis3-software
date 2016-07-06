#include <ros/ros.h>
#include <simulation/RobotSim.h>
#include <messages/ActuatorOut.h>
#include <messages/NavFilterOut.h>
#include <messages/RobotPose.h>
#include <messages/GrabberFeedback.h>
#include <messages/SimControl.h>
#include <messages/nb1_to_i7_msg.h>
#include <messages/CollisionOut.h>
#include <messages/CVSearchCmd.h>
#include <messages/CVSamplesFound.h>
#include <messages/KeyframeList.h>
#include <messages/CreateROIKeyframe.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <robot_control/map_layers.h>

void actuatorCallback(const messages::ActuatorOut::ConstPtr& msg);
void simControlCallback(const messages::SimControl::ConstPtr& msg);
bool cvSearchCmdCallback(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res);
bool createROIKeyframeCallback(messages::CreateROIKeyframe::Request &req, messages::CreateROIKeyframe::Response &res);
void gridMapAddLayers(int layerStartIndex, int layerEndIndex, grid_map::GridMap &map);
void publishKeyframeList();

ros::Publisher cvSamplesFoundPub;
messages::ActuatorOut actuatorCmd;
messages::CollisionOut collisionMsgOut;
messages::CVSamplesFound cvSamplesFoundMsgOut;
RobotSim robotSim(0.0, 0.0, 0.0,20);
bool cvFindSample = false;
messages::CVSampleProps cvSampleProps;
messages::KeyframeList keyframeListMsg;
grid_map::GridMap keyframe;
ros::Publisher keyframeListPub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simulation_node");
    ros::NodeHandle nh;
    ros::Subscriber actuatorSub = nh.subscribe<messages::ActuatorOut>("control/actuatorout/all",1,actuatorCallback);
    ros::Subscriber simConSub = nh.subscribe<messages::SimControl>("simulation/simcontrol/simcontrol",1,simControlCallback);
    ros::Publisher navPub = nh.advertise<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout",1);
    ros::Publisher posePub = nh.advertise<messages::RobotPose>("/hsm/masterexec/globalpose",1);
    ros::Publisher grabberPub = nh.advertise<messages::GrabberFeedback>("roboteq/grabberin/grabberin",1);
    ros::Publisher nb1Pub = nh.advertise<messages::nb1_to_i7_msg>("hw_interface/nb1in/nb1in",1);
    ros::Publisher collisionPub = nh.advertise<messages::CollisionOut>("lidar/collisiondetectionout/collisiondetectionout", 1);
    cvSamplesFoundPub = nh.advertise<messages::CVSamplesFound>("vision/samplesearch/samplesearchout", 1);
    keyframeListPub = nh.advertise<messages::KeyframeList>("/slam/keyframesnode/keyframelist", 1);
    ros::ServiceServer cvSearchCmdServ = nh.advertiseService("/vision/samplesearch/searchforsamples", cvSearchCmdCallback);
    ros::ServiceServer createROIKeyframeServ = nh.advertiseService("/slam/keyframesnode/createroikeyframe", createROIKeyframeCallback);
    messages::NavFilterOut navMsgOut;
    messages::RobotPose poseMsgOut;
    messages::GrabberFeedback grabberMsgOut;
    messages::nb1_to_i7_msg nb1MsgOut;


    double linV; // m/s
    double angV; // deg/s
    const double linVGain = 1.2/1000.0/6.0; // m/s per speed cmd
    const double angVGain = 45.0/2650.0; // deg/s per speec cmd
    actuatorCmd.grabber_stop_cmd = 0;
    actuatorCmd.slide_pos_cmd = 1000;
    actuatorCmd.drop_pos_cmd = -1000;

    keyframe.setFrameId("map");
    keyframe.setGeometry(grid_map::Length(150.0, 150.0), 1.0, grid_map::Position(0.0, 0.0));
    gridMapAddLayers(MAP_KEYFRAME_LAYERS_START_INDEX, MAP_KEYFRAME_LAYERS_END_INDEX, keyframe);

    ros::Rate loopRate(20);
    while(ros::ok())
    {
        linV = linVGain*(actuatorCmd.fl_speed_cmd + actuatorCmd.fr_speed_cmd + actuatorCmd.ml_speed_cmd + actuatorCmd.mr_speed_cmd + actuatorCmd.bl_speed_cmd + actuatorCmd.br_speed_cmd);
        angV = angVGain*(actuatorCmd.fl_speed_cmd - actuatorCmd.fr_speed_cmd + actuatorCmd.ml_speed_cmd - actuatorCmd.mr_speed_cmd + actuatorCmd.bl_speed_cmd - actuatorCmd.br_speed_cmd);
        //ROS_DEBUG("linV: %f",linV);
        //ROS_DEBUG("angV: %f",angV);
        robotSim.drive(linV, angV);
        robotSim.runGrabber(actuatorCmd.slide_pos_cmd, actuatorCmd.drop_pos_cmd, actuatorCmd.grabber_stop_cmd, actuatorCmd.grabber_stop_cmd);
        navMsgOut.x_position = robotSim.xPos;
        navMsgOut.y_position = robotSim.yPos;
        navMsgOut.velocity = linV;
        navMsgOut.yaw_rate = angV;
        navMsgOut.heading = robotSim.heading;
        navMsgOut.human_heading = fmod(robotSim.heading, 360.0);
        poseMsgOut.x = robotSim.xPos;
        poseMsgOut.y = robotSim.yPos;
        poseMsgOut.heading = robotSim.heading;
        poseMsgOut.humanHeading = fmod(robotSim.heading, 360.0);
        poseMsgOut.northAngle = 90.0;
        grabberMsgOut.sliderPosAvg = robotSim.slidePos;
        grabberMsgOut.dropPos = robotSim.dropPos;
        grabberMsgOut.slideStatus = robotSim.slideStop;
        grabberMsgOut.dropStatus = robotSim.dropStop;
        nb1MsgOut.pause_switch = robotSim.nb1PauseSwitch;
        navPub.publish(navMsgOut);
        posePub.publish(poseMsgOut);
        grabberPub.publish(grabberMsgOut);
        nb1Pub.publish(nb1MsgOut);
        collisionPub.publish(collisionMsgOut);
        loopRate.sleep();
        ros::spinOnce();
    }
    return 0;
}

void actuatorCallback(const messages::ActuatorOut::ConstPtr& msg)
{
    actuatorCmd = *msg;
}

void simControlCallback(const messages::SimControl::ConstPtr& msg)
{
    if(msg->teleport)
    {
        robotSim.teleport(msg->teleX, msg->teleY, msg->teleHeading);
    }
    if(msg->setSimSpeed)
    {
        if(msg->simSpeed>0.0) robotSim.dt = robotSim.normalSpeedDT*msg->simSpeed;
    }
    if(msg->pauseSwitch) robotSim.nb1PauseSwitch = 255;
    else robotSim.nb1PauseSwitch = 0;
    collisionMsgOut.collision = msg->collision;
    collisionMsgOut.distance_to_collision = msg->collisionDistance;
    cvFindSample = msg->cvFindSample;
    cvSampleProps.type = msg->cvType;
    cvSampleProps.distance = msg->cvDistance;
    cvSampleProps.bearing = msg->cvBearing;
    cvSampleProps.confidence = msg->cvConfidence;
    if(msg->pubKeyframeList) publishKeyframeList();
}

bool cvSearchCmdCallback(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res)
{
    cvSamplesFoundMsgOut.procType = req.procType;
    cvSamplesFoundMsgOut.serialNum = req.serialNum;
    cvSamplesFoundMsgOut.sampleList.clear();
    if(cvFindSample)
    {
        cvSamplesFoundMsgOut.sampleList.push_back(cvSampleProps);
    }
    cvSamplesFoundPub.publish(cvSamplesFoundMsgOut);
    ros::spinOnce();
    return true;
}

bool createROIKeyframeCallback(messages::CreateROIKeyframe::Request &req, messages::CreateROIKeyframe::Response &res)
{
    keyframe.add(layerToString(_keyframeDriveability), 0.0);
    keyframe.add(layerToString(_keyframeDriveabilityConf), 0.9);
    keyframe.atPosition(layerToString(_keyframeDriveability), grid_map::Position(30.0, 30.0)) = 1;
    keyframe.atPosition(layerToString(_keyframeDriveability), grid_map::Position(50.0, 0.0)) = 2;
    keyframe.atPosition(layerToString(_keyframeDriveability), grid_map::Position(30.0, -30.0)) = 2;
    grid_map::GridMapRosConverter::toMessage(keyframe, res.keyframe.map);
    res.keyframe.x = robotSim.xPos;
    res.keyframe.y = robotSim.yPos;
    res.keyframe.heading = robotSim.heading;
    res.keyframe.associatedROI = req.roiIndex;
    return true;
}

void gridMapAddLayers(int layerStartIndex, int layerEndIndex, grid_map::GridMap &map)
{
    for(int i=layerStartIndex; i<=layerEndIndex; i++)
    {
        map.add(layerToString(static_cast<MAP_LAYERS_T>(i)));
    }
}

void publishKeyframeList()
{
    keyframeListMsg.keyframeList.clear();
    //keyframeListMsg.keyframeList.resize(1);
    keyframeListMsg.keyframeList.resize(2);

    keyframe.add(layerToString(_keyframeDriveability), 0.0);
    keyframe.add(layerToString(_keyframeDriveabilityConf), 0.9);
    keyframe.add(layerToString(_keyframeObjectHeight), 0.0);
    keyframe.atPosition(layerToString(_keyframeDriveability), grid_map::Position(30.0, 30.0)) = 1;
    keyframe.atPosition(layerToString(_keyframeDriveability), grid_map::Position(50.0, 0.0)) = 2;
    keyframe.atPosition(layerToString(_keyframeDriveability), grid_map::Position(30.0, -30.0)) = 2;
    grid_map::GridMapRosConverter::toMessage(keyframe, keyframeListMsg.keyframeList.at(0).map);
    keyframeListMsg.keyframeList.at(0).x = 20.0;
    keyframeListMsg.keyframeList.at(0).y = 15.0;
    keyframeListMsg.keyframeList.at(0).heading = 0.0;

    keyframe.add(layerToString(_keyframeDriveability), 0.0);
    keyframe.add(layerToString(_keyframeDriveabilityConf), 0.9);
    keyframe.add(layerToString(_keyframeObjectHeight), 0.0);
    keyframe.atPosition(layerToString(_keyframeDriveability), grid_map::Position(30.0, 30.0)) = 1;
    keyframe.atPosition(layerToString(_keyframeDriveability), grid_map::Position(50.0, 0.0)) = 2;
    keyframe.atPosition(layerToString(_keyframeDriveability), grid_map::Position(30.0, -30.0)) = 2;
    grid_map::GridMapRosConverter::toMessage(keyframe, keyframeListMsg.keyframeList.at(1).map);
    keyframeListMsg.keyframeList.at(1).x = 20.0;
    keyframeListMsg.keyframeList.at(1).y = 15.0;
    keyframeListMsg.keyframeList.at(1).heading = 45.0;

    keyframeListPub.publish(keyframeListMsg);
}
