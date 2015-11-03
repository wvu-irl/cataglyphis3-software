#include <ros/ros.h>
#include <simulation/RobotSim.h>
#include <messages/ActuatorOut.h>
#include <messages/NavFilterOut.h>
#include <messages/GrabberFeedback.h>

void actuatorCallback(const messages::ActuatorOut::ConstPtr& msg);

messages::ActuatorOut actuatorCmd;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simulation_node");
    ros::NodeHandle nh;
    ros::Subscriber actuatorSub = nh.subscribe<messages::ActuatorOut>("control/actuatorout/all",1,actuatorCallback);
    ros::Publisher navPub = nh.advertise<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout",1);
    ros::Publisher grabberPub = nh.advertise<messages::GrabberFeedback>("roboteq/grabberin/grabberin",1);
    messages::NavFilterOut navMsgOut;
    messages::GrabberFeedback grabberMsgOut;
    RobotSim robotSim(0.0, 0.0, 0.0,20);
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
        navMsgOut.heading = robotSim.heading;
        navMsgOut.human_heading = fmod(robotSim.heading, 360.0);
        grabberMsgOut.sliderPosAvg = robotSim.slidePos;
        grabberMsgOut.dropPos = robotSim.dropPos;
        grabberMsgOut.slideStatus = robotSim.slideStop;
        grabberMsgOut.dropStatus = robotSim.dropStop;
        navPub.publish(navMsgOut);
        grabberPub.publish(grabberMsgOut);
        loopRate.sleep();
        ros::spinOnce();
    }
    return 0;
}

void actuatorCallback(const messages::ActuatorOut::ConstPtr& msg)
{
    actuatorCmd = *msg;
}
