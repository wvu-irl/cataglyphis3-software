#include <ros/ros.h>
#include <simulation/RobotSim.h>
#include <messages/ActuatorOut.h>
#include <messages/NavFilterOut.h>
#include <messages/GrabberFeedback.h>
#include <messages/SimControl.h>
#include <messages/nb1_to_i7_msg.h>
#include <messages/CollisionOut.h>
#include <messages/CVSearchCmd.h>
#include <messages/CVSamplesFound.h>

void actuatorCallback(const messages::ActuatorOut::ConstPtr& msg);
void simControlCallback(const messages::SimControl::ConstPtr& msg);
bool cvSearchCmdCallback(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res);

ros::Publisher cvSamplesFoundPub;
messages::ActuatorOut actuatorCmd;
messages::CollisionOut collisionMsgOut;
messages::CVSamplesFound cvSamplesFoundMsgOut;
RobotSim robotSim(0.0, 0.0, 0.0,20);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simulation_node");
    ros::NodeHandle nh;
    ros::Subscriber actuatorSub = nh.subscribe<messages::ActuatorOut>("control/actuatorout/all",1,actuatorCallback);
    ros::Subscriber simConSub = nh.subscribe<messages::SimControl>("simulation/simcontrol/simcontrol",1,simControlCallback);
    ros::Publisher navPub = nh.advertise<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout",1);
    ros::Publisher grabberPub = nh.advertise<messages::GrabberFeedback>("roboteq/grabberin/grabberin",1);
    ros::Publisher nb1Pub = nh.advertise<messages::nb1_to_i7_msg>("hw_interface/nb1in/nb1in",1);
    ros::Publisher collisionPub = nh.advertise<messages::CollisionOut>("lidar/collisiondetectionout/collisiondetectionout", 1);
    cvSamplesFoundPub = nh.advertise<messages::CVSamplesFound>("vision/samplesearch/samplesearchout", 1);
    ros::ServiceServer cvSearchCmdServ = nh.advertiseService("/vision/samplesearch/searchforsamples", cvSearchCmdCallback);
    messages::NavFilterOut navMsgOut;
    messages::GrabberFeedback grabberMsgOut;
    messages::nb1_to_i7_msg nb1MsgOut;


    double linV; // m/s
    double angV; // deg/s
    const double linVGain = 1.2/1000.0/6.0; // m/s per speed cmd
    const double angVGain = 45.0/2650.0; // deg/s per speec cmd
    actuatorCmd.grabber_stop_cmd = 0;
    actuatorCmd.slide_pos_cmd = 1000;
    actuatorCmd.drop_pos_cmd = -1000;
    ros::Rate loopRate(20);
    while(ros::ok())
    {
        linV = linVGain*(actuatorCmd.fl_speed_cmd + actuatorCmd.fr_speed_cmd + actuatorCmd.ml_speed_cmd + actuatorCmd.mr_speed_cmd + actuatorCmd.bl_speed_cmd + actuatorCmd.br_speed_cmd);
        angV = angVGain*(actuatorCmd.fl_speed_cmd - actuatorCmd.fr_speed_cmd + actuatorCmd.ml_speed_cmd - actuatorCmd.mr_speed_cmd + actuatorCmd.bl_speed_cmd - actuatorCmd.br_speed_cmd);
        ROS_DEBUG("linV: %f",linV);
        ROS_DEBUG("angV: %f",angV);
        robotSim.drive(linV, angV);
        robotSim.runGrabber(actuatorCmd.slide_pos_cmd, actuatorCmd.drop_pos_cmd, actuatorCmd.grabber_stop_cmd, actuatorCmd.grabber_stop_cmd);
        navMsgOut.x_position = robotSim.xPos;
        navMsgOut.y_position = robotSim.yPos;
        navMsgOut.velocity = linV;
        navMsgOut.yaw_rate = angV;
        navMsgOut.heading = robotSim.heading;
        navMsgOut.human_heading = fmod(robotSim.heading, 360.0);
        grabberMsgOut.sliderPosAvg = robotSim.slidePos;
        grabberMsgOut.dropPos = robotSim.dropPos;
        grabberMsgOut.slideStatus = robotSim.slideStop;
        grabberMsgOut.dropStatus = robotSim.dropStop;
        nb1MsgOut.pause_switch = robotSim.nb1PauseSwitch;
        navPub.publish(navMsgOut);
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
}

bool cvSearchCmdCallback(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res)
{
    cvSamplesFoundMsgOut.procType = req.procType;
    cvSamplesFoundMsgOut.serialNum = req.serialNum;
    cvSamplesFoundMsgOut.sampleList.clear();
    cvSamplesFoundPub.publish(cvSamplesFoundMsgOut);
    ros::spinOnce();
    return true;
}
