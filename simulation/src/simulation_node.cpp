/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>
#include <simulation/RobotSim.h>
#include <messages/ActuatorOut.h>
#include <messages/NavFilterOut.h>
#include <messages/SLAMPoseOut.h>
#include <messages/GrabberFeedback.h>
#include <messages/SimControl.h>
#include <messages/nb1_to_i7_msg.h>
#include <messages/CollisionOut.h>
#include <messages/CVSearchCmd.h>
#include <messages/CVSamplesFound.h>
#include <messages/KeyframeList.h>
#include <messages/CreateROIHazardMap.h>
#include <messages/NavFilterControl.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <robot_control/map_layers.h>

void actuatorCallback(const messages::ActuatorOut::ConstPtr& msg);
void simControlCallback(const messages::SimControl::ConstPtr& msg);
bool cvSearchCmdCallback(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res);
bool createROIHazardMapCallback(messages::CreateROIHazardMap::Request &req, messages::CreateROIHazardMap::Response &res);
bool navControlCallback(messages::NavFilterControl::Request &req, messages::NavFilterControl::Response &res);
void gridMapAddLayers(int layerStartIndex, int layerEndIndex, grid_map::GridMap &map);
void publishKeyframeList();
void biasRemovalTimerCallback(const ros::TimerEvent &event);

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
float rollAngle = 0.0;
ros::Timer biasRemovalTimer;
bool biasRemovalFinished = false;
bool homingUpdated = false;
bool navStopRequest = false;
float northAngle = 90.0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simulation_node");
    ros::NodeHandle nh;
    ros::Subscriber actuatorSub = nh.subscribe<messages::ActuatorOut>("control/actuatorout/all",1,actuatorCallback);
    ros::Subscriber simConSub = nh.subscribe<messages::SimControl>("simulation/simcontrol/simcontrol",1,simControlCallback);
    ros::Publisher navPub = nh.advertise<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout",1);
    ros::Publisher slamPosePub = nh.advertise<messages::SLAMPoseOut>("/slam/localizationnode/slamposeout",1);
    ros::Publisher grabberPub = nh.advertise<messages::GrabberFeedback>("roboteq/grabberin/grabberin",1);
    ros::Publisher nb1Pub = nh.advertise<messages::nb1_to_i7_msg>("hw_interface/nb1in/nb1in",1);
    ros::Publisher collisionPub = nh.advertise<messages::CollisionOut>("lidar/collisiondetectionout/collisiondetectionout", 1);
    cvSamplesFoundPub = nh.advertise<messages::CVSamplesFound>("vision/samplesearch/samplesearchout", 1);
    keyframeListPub = nh.advertise<messages::KeyframeList>("/slam/keyframesnode/keyframelist", 1);
    ros::ServiceServer cvSearchCmdServ = nh.advertiseService("/vision/samplesearch/searchforsamples", cvSearchCmdCallback);
    ros::ServiceServer createROIHazardMapServ = nh.advertiseService("/lidar/collisiondetection/createroihazardmap", createROIHazardMapCallback);
    ros::ServiceServer navControlCallbackServ = nh.advertiseService("/navigation/navigationfilter/control", navControlCallback);
    biasRemovalTimer = nh.createTimer(ros::Duration(5.0), biasRemovalTimerCallback);
    biasRemovalTimer.stop();
    messages::NavFilterOut navMsgOut;
    messages::SLAMPoseOut slamPoseMsgOut;
    messages::GrabberFeedback grabberMsgOut;
    messages::nb1_to_i7_msg nb1MsgOut;


    double linV; // m/s
    double angV; // deg/s
    const double linVGain = 1.4/900.0/6.0; // m/s per speed cmd
    const double angVGain = 45.0/2650.0; // deg/s per speec cmd
    actuatorCmd.grabber_stop_cmd = 0;
    actuatorCmd.slide_pos_cmd = 1000;
    actuatorCmd.drop_pos_cmd = -1000;

    cvSampleProps.white = true;
    cvSampleProps.blueOrPurple = true;

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
        navMsgOut.north_angle = northAngle;
        navMsgOut.heading = robotSim.heading;
        navMsgOut.human_heading = fmod(robotSim.heading, 360.0);
        navMsgOut.roll = rollAngle;
        navMsgOut.nav_status = biasRemovalFinished;
        navMsgOut.homing_updated = homingUpdated;
        navMsgOut.stop_request = navStopRequest;
        navMsgOut.nb1_good = true;
        slamPoseMsgOut.globalX = robotSim.xPos;
        slamPoseMsgOut.globalY = robotSim.yPos;
        slamPoseMsgOut.globalHeading = robotSim.heading;
        grabberMsgOut.sliderPosAvg = robotSim.slidePos;
        grabberMsgOut.dropPos = robotSim.dropPos;
        grabberMsgOut.slideStatus = robotSim.slideStop;
        grabberMsgOut.dropStatus = robotSim.dropStop;
        nb1MsgOut.pause_switch = robotSim.nb1PauseSwitch;
        navPub.publish(navMsgOut);
        slamPosePub.publish(slamPoseMsgOut);
        grabberPub.publish(grabberMsgOut);
        nb1Pub.publish(nb1MsgOut);
        collisionPub.publish(collisionMsgOut);
        biasRemovalFinished = false;
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
    collisionMsgOut.distance_to_drive = msg->avoidDriveDistance;
    collisionMsgOut.angle_to_drive = msg->avoidDriveAngle;
    cvFindSample = msg->cvFindSample;
    cvSampleProps.distance = msg->cvDistance;
    cvSampleProps.bearing = msg->cvBearing;
    cvSampleProps.confidence = msg->cvConfidence;
    if(msg->pubKeyframeList) publishKeyframeList();
    rollAngle = msg->rollAngle;
    homingUpdated = msg->homingUpdated;
    navStopRequest = msg->navStopRequest;
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

bool createROIHazardMapCallback(messages::CreateROIHazardMap::Request &req, messages::CreateROIHazardMap::Response &res)
{
    res.x_mean.resize(3);
    res.y_mean.resize(3);
    res.x_mean.at(0) = 15.0;
    res.y_mean.at(0) = 15.0;
    res.x_mean.at(1) = 20.0;
    res.y_mean.at(1) = 0.0;
    res.x_mean.at(2) = 15.0;
    res.y_mean.at(2) = -3.0;
    return true;
}

bool navControlCallback(messages::NavFilterControl::Request &req, messages::NavFilterControl::Response &res)
{
    if(req.runBiasRemoval) biasRemovalTimer.start();
    if(req.setNorthAngle) northAngle = req.northAngle;
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

void biasRemovalTimerCallback(const ros::TimerEvent &event)
{
    biasRemovalFinished = true;
    biasRemovalTimer.stop();
}
