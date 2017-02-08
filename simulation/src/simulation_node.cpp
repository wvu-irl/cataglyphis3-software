#include <ros/ros.h>
#include <simulation/RobotSim.h>
#include <messages/ActuatorOut.h>
#include <messages/NavFilterOut.h>
#include <messages/SLAMPoseOut.h>
#include <messages/GrabberFeedback.h>
#include <messages/SimControl.h>
#include <messages/nb1_to_i7_msg.h>
#include <messages/nb2_3_to_i7_msg.h>
#include <messages/CollisionOut.h>
#include <messages/CVSearchCmd.h>
#include <messages/CVSamplesFound.h>
#include <messages/KeyframeList.h>
#include <messages/CreateROIHazardMap.h>
#include <messages/NavFilterControl.h>
#include <messages/SimSampleLocations.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <robot_control/map_layers.h>
#include <robot_control/ROIList.h>
#include <chrono>
#include <random>

//#define MANUAL_CV_CONTROL
#define NUM_SAMPLES 4
//bool roisWithSample[15] = {1,1,0,1,1,0,1,0,1,1,1,1,0,0,1};
bool roisWithSample[7] = {1,1,0,0,1,0,1};

void actuatorCallback(const messages::ActuatorOut::ConstPtr& msg);
void simControlCallback(const messages::SimControl::ConstPtr& msg);
void roisModifiedCallback(const robot_control::ROIList::ConstPtr& msg);
bool cvSearchCmdCallback(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res);
bool createROIHazardMapCallback(messages::CreateROIHazardMap::Request &req, messages::CreateROIHazardMap::Response &res);
bool navControlCallback(messages::NavFilterControl::Request &req, messages::NavFilterControl::Response &res);
void gridMapAddLayers(int layerStartIndex, int layerEndIndex, grid_map::GridMap &map);
void publishKeyframeList();
void biasRemovalTimerCallback(const ros::TimerEvent &event);
void setSampleLocations();
void rotateCoord(float origX, float origY, float &newX, float &newY, float angleDeg);

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
bool sampleLocationsInitialized = false;
robot_control::ROIList roiList;
std::vector<std::pair<float, float>> sampleLocations;
bool samplesGrabbed[NUM_SAMPLES] = {false};
bool grabAttemptPrev = false;
float goodGrabThreshold = -0.2; // 0.2
ros::Publisher sampleLocationsPub;
messages::SimSampleLocations sampleLocationsMsg;

int main(int argc, char** argv)
{
    srand(time(NULL));
    ros::init(argc, argv, "simulation_node");
    ros::NodeHandle nh;
    ros::Subscriber actuatorSub = nh.subscribe<messages::ActuatorOut>("control/actuatorout/all",1,actuatorCallback);
    ros::Subscriber simConSub = nh.subscribe<messages::SimControl>("simulation/simcontrol/simcontrol",1,simControlCallback);
    ros::Subscriber roisModifiedSub = nh.subscribe<robot_control::ROIList>("/control/mapmanager/roimodifiedlist",1,roisModifiedCallback);
    ros::Publisher navPub = nh.advertise<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout",1);
    ros::Publisher slamPosePub = nh.advertise<messages::SLAMPoseOut>("/slam/localizationnode/slamposeout",1);
    ros::Publisher grabberPub = nh.advertise<messages::GrabberFeedback>("roboteq/grabberin/grabberin",1);
    ros::Publisher nb1Pub = nh.advertise<messages::nb1_to_i7_msg>("hw_interface/nb1in/nb1in",1);
    ros::Publisher nb2Pub = nh.advertise<messages::nb2_3_to_i7_msg>("hw_interface/nb2in/nb2in",1);
    ros::Publisher collisionPub = nh.advertise<messages::CollisionOut>("lidar/collisiondetectionout/collisiondetectionout", 1);
    cvSamplesFoundPub = nh.advertise<messages::CVSamplesFound>("vision/samplesearch/samplesearchout", 1);
    keyframeListPub = nh.advertise<messages::KeyframeList>("/slam/keyframesnode/keyframelist", 1);
    sampleLocationsPub = nh.advertise<messages::SimSampleLocations>("/simulation/simulator/sampletruthlocations", 1);
    ros::ServiceServer cvSearchCmdServ = nh.advertiseService("/vision/samplesearch/searchforsamples", cvSearchCmdCallback);
    ros::ServiceServer createROIHazardMapServ = nh.advertiseService("/lidar/collisiondetection/createroihazardmap", createROIHazardMapCallback);
    ros::ServiceServer navControlCallbackServ = nh.advertiseService("/navigation/navigationfilter/control", navControlCallback);
    biasRemovalTimer = nh.createTimer(ros::Duration(5.0), biasRemovalTimerCallback);
    biasRemovalTimer.stop();
    messages::NavFilterOut navMsgOut;
    messages::SLAMPoseOut slamPoseMsgOut;
    messages::GrabberFeedback grabberMsgOut;
    messages::nb1_to_i7_msg nb1MsgOut;
    messages::nb2_3_to_i7_msg nb2MsgOut;


    double linV; // m/s
    double angV; // deg/s
    const double linVGain = 1.4/900.0/6.0; // m/s per speed cmd
    const double angVGain = 45.0/2650.0; // deg/s per speec cmd
    float distanceToSample;
    float minDistanceToSample;
    int minDistanceToSampleIndex;
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
        if(robotSim.grabAttempt && !grabAttemptPrev)
        {
            ROS_INFO("grab attempt");
            if(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) > goodGrabThreshold)
            {
                ROS_INFO("good grab");
                minDistanceToSample = 10000.0;
                minDistanceToSampleIndex = 0;
                for(int i=0; i<NUM_SAMPLES; i++)
                {
                    distanceToSample = hypot(sampleLocations.at(i).first - robotSim.xPos, sampleLocations.at(i).second - robotSim.yPos);
                    if(distanceToSample<minDistanceToSample) {minDistanceToSample = distanceToSample; minDistanceToSampleIndex = i;}
                }
                if(!samplesGrabbed[minDistanceToSampleIndex]) {samplesGrabbed[minDistanceToSampleIndex] = true; ROS_INFO("sample %i grabbed",minDistanceToSampleIndex);}
            }
        }
        grabAttemptPrev = robotSim.grabAttempt;
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
        navMsgOut.nb1_good = 1;
        navMsgOut.nb2_good = 1;
        slamPoseMsgOut.globalX = robotSim.xPos;
        slamPoseMsgOut.globalY = robotSim.yPos;
        slamPoseMsgOut.globalHeading = robotSim.heading;
        grabberMsgOut.sliderPosAvg = robotSim.slidePos;
        grabberMsgOut.dropPos = robotSim.dropPos;
        grabberMsgOut.slideStatus = robotSim.slideStop;
        grabberMsgOut.dropStatus = robotSim.dropStop;
        nb1MsgOut.pause_switch = robotSim.nb1PauseSwitch;
        nb2MsgOut.pause_switch = robotSim.nb2PauseSwitch;
        navPub.publish(navMsgOut);
        slamPosePub.publish(slamPoseMsgOut);
        grabberPub.publish(grabberMsgOut);
        nb1Pub.publish(nb1MsgOut);
        nb2Pub.publish(nb2MsgOut);
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
    if(msg->pauseSwitch) {robotSim.nb1PauseSwitch = 255; robotSim.nb2PauseSwitch = 255;}
    else {robotSim.nb1PauseSwitch = 0; robotSim.nb2PauseSwitch = 0;}
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

void roisModifiedCallback(const robot_control::ROIList::ConstPtr &msg)
{
    roiList = *msg;
    if(!sampleLocationsInitialized)
    {
        sampleLocations.resize(roiList.ROIList.size());
        sampleLocationsMsg.sampleX.resize(roiList.ROIList.size(),0.0);
        sampleLocationsMsg.sampleY.resize(roiList.ROIList.size(),0.0);
        sampleLocationsMsg.sampleInROI.resize(roiList.ROIList.size(),false);
        setSampleLocations();
        sampleLocationsInitialized = true;
    }
}

bool cvSearchCmdCallback(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res)
{
    float sampleProb;
    float distanceToSample;
    float randomProbNoise;
    float sampleBodyX;
    float sampleBodyY;
    const float maxSampleProb = 0.99;
    const float maxDetectionDist = 5.0; // m
    const float randomGain = 0.3;
    const float randomRange = 1.5;
    const float randomOffset = -1.0;
    cvSamplesFoundMsgOut.procType = req.procType;
    cvSamplesFoundMsgOut.serialNum = req.serialNum;
    cvSamplesFoundMsgOut.sampleList.clear();
#ifdef MANUAL_CV_CONTROL
    if(cvFindSample)
    {
        cvSamplesFoundMsgOut.sampleList.push_back(cvSampleProps);
    }
#else

    for(int i=0; i<NUM_SAMPLES; i++)
    {
        if(!samplesGrabbed[i])
        {
            distanceToSample = hypot(sampleLocations.at(i).first - robotSim.xPos, sampleLocations.at(i).second - robotSim.yPos);
            randomProbNoise = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * randomRange + randomOffset;
            sampleProb = maxSampleProb - pow(distanceToSample/maxDetectionDist,2.0);
            //if(sampleProb>0.0) sampleProb += randomGain*randomProbNoise;
            if(sampleProb>1.0) sampleProb = 1.0;
            else if(sampleProb<0.0) sampleProb = 0.0;
            if(sampleProb>0.0)
            {
                if((static_cast<float>(rand()) / static_cast<float>(RAND_MAX))<=sampleProb) cvSampleProps.confidence = 0.9;
                else cvSampleProps.confidence = 0.0;
                cvSampleProps.distance = distanceToSample;
                rotateCoord(sampleLocations.at(i).first - robotSim.xPos, sampleLocations.at(i).second - robotSim.yPos, sampleBodyX, sampleBodyY, fmod(robotSim.heading,360.0));
                cvSampleProps.bearing = RAD2DEG*atan2(sampleBodyY, sampleBodyX);
                cvSamplesFoundMsgOut.sampleList.push_back(cvSampleProps);
            }
        }
    }
#endif // MANUAL_CV_CONTROL
    if(robotSim.grabAttempt) robotSim.grabAttempt = false;
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

void setSampleLocations()
{
    float xLocal;
    float yLocal;
    float radialDist;
    float angle;
    float angleToROI;
    int roiListLen = roiList.ROIList.size();
    int j=0;
    const float biasStdDev = 6.0; // m
    const float biasMaxDist = 8.0; // m
    float biasXLocal;
    float biasYLocal;
    const float samplePlacementStdDev = 12.0;
    unsigned biasSeed = std::chrono::system_clock::now().time_since_epoch().count();
    bool keepFindingRandomBias = true;
    bool keepFindingSamplePos = true;
    std::default_random_engine gen(biasSeed);

    for(int i=0; i<roiListLen; i++)
    {
        if(roisWithSample[i])
        {
            std::normal_distribution<float> biasDist(0.0,biasStdDev);
            for(int j=0; j<2; j++)
            {
                keepFindingRandomBias = true;
                while(keepFindingRandomBias)
                {
                    if(j==0)
                    {
                        biasXLocal = biasDist(gen);
                        if(fabs(biasXLocal)>biasMaxDist) keepFindingRandomBias = true;
                        else keepFindingRandomBias = false;
                    }
                    else
                    {
                        biasYLocal = biasDist(gen);
                        if(fabs(biasYLocal)>biasMaxDist) keepFindingRandomBias = true;
                        else keepFindingRandomBias = false;
                    }
                }
            }
            std::normal_distribution<float> sampleXDist(biasXLocal,samplePlacementStdDev);
            std::normal_distribution<float> sampleYDist(biasYLocal,samplePlacementStdDev);
            for(int j=0; j<2; j++)
            {
                keepFindingSamplePos = true;
                while(keepFindingSamplePos)
                {
                    if(j==0)
                    {
                        xLocal = sampleXDist(gen);
                        if(fabs(xLocal)>(std::min(roiList.ROIList.at(i).radialAxis,roiList.ROIList.at(i).tangentialAxis)-fabs(biasXLocal))) keepFindingSamplePos = true;
                        else keepFindingSamplePos = false;
                    }
                    else
                    {
                        yLocal = sampleYDist(gen);
                        if(fabs(yLocal)>(std::min(roiList.ROIList.at(i).radialAxis,roiList.ROIList.at(i).tangentialAxis)-fabs(biasYLocal))) keepFindingSamplePos = true;
                        else keepFindingSamplePos = false;
                    }
                }
            }
            rotateCoord(xLocal, yLocal, sampleLocations.at(j).first, sampleLocations.at(j).second, -angleToROI);
            sampleLocations.at(j).first += roiList.ROIList.at(i).x;
            sampleLocations.at(j).second += roiList.ROIList.at(i).y;
            ROS_INFO("roi = [%f,%f]",roiList.ROIList.at(i).x,roiList.ROIList.at(i).y);
            ROS_INFO("sample location %i = [%f,%f]",i,sampleLocations.at(j).first,sampleLocations.at(j).second);
            sampleLocationsMsg.sampleX.at(i) = sampleLocations.at(j).first;
            sampleLocationsMsg.sampleY.at(i) = sampleLocations.at(j).second;
            sampleLocationsMsg.sampleInROI.at(i) = true;
            j++;
        }
    }
    sampleLocationsPub.publish(sampleLocationsMsg);
}

void rotateCoord(float origX, float origY, float &newX, float &newY, float angleDeg)
{
    newX = origX*cos(DEG2RAD*angleDeg)+origY*sin(DEG2RAD*angleDeg);
    newY = -origX*sin(DEG2RAD*angleDeg)+origY*cos(DEG2RAD*angleDeg);
}
