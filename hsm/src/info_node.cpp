#include <ros/ros.h>
#include <math.h>

#include <comm/Servo_status.h>
#include <comm/nb1_to_i7_msg.h>
#include <comm/nb2_3_to_i7_msg.h>
#include <roboteq_interface/encoder_data.h>
#include <roboteq_interface/GrabberFeedback.h>
#include <navigation/NavFilterOut.h>
#include <robot_control/ExecStateMachineInfo.h>
#include <robot_control/MissionPlanningInfo.h>
#include <computer_vision/ComputerVisionOut.h>

comm::Servo_status cameraServo;
comm::Servo_status laserServo;
comm::nb1_to_i7_msg nb1;
comm::nb2_3_to_i7_msg nb2;
comm::nb2_3_to_i7_msg nb3;
roboteq_interface::encoder_data left;
roboteq_interface::encoder_data right;
roboteq_interface::GrabberFeedback grabber;
navigation::NavFilterOut nav;
robot_control::ExecStateMachineInfo exec;
robot_control::MissionPlanningInfo mis;
computer_vision::ComputerVisionOut vision;


void cameraServoCallback(const comm::Servo_status::ConstPtr& msg);
void laserServoCallback(const comm::Servo_status::ConstPtr& msg);

void nb1Callback(const comm::nb1_to_i7_msg::ConstPtr& msg);
void nb2Callback(const comm::nb2_3_to_i7_msg::ConstPtr& msg);
void nb3Callback(const comm::nb2_3_to_i7_msg::ConstPtr& msg);

void leftDriveRoboteqCallback(const roboteq_interface::encoder_data::ConstPtr& msg);
void rightDriveRoboteqCallback(const roboteq_interface::encoder_data::ConstPtr& msg);
void grabberRoboteqCallback(const roboteq_interface::GrabberFeedback::ConstPtr& msg);

void navFilterCallback(const navigation::NavFilterOut::ConstPtr& msg);

void cvCallback(const computer_vision::ComputerVisionOut::ConstPtr& msg);

void execCallback(const robot_control::ExecStateMachineInfo::ConstPtr& msg);
void misCallback(const robot_control::MissionPlanningInfo::ConstPtr& msg);


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "info_node");
    ros::NodeHandle nh;
    ROS_INFO("Info Node started");
    
    ros::Rate loopRate(50);
    
    ros::Subscriber cameraServoStatus = nh.subscribe<comm::Servo_status>("camera_pan_servo_status", 1, cameraServoCallback);
    ros::Subscriber laserServoStatus = nh.subscribe<comm::Servo_status>("laser_scanner_servo_status", 1, laserServoCallback);
    
    ros::Subscriber grabberSub = nh.subscribe<roboteq_interface::GrabberFeedback>("roboteq/grabberin/grabberin", 1, grabberRoboteqCallback);
    ros::Subscriber leftDriveSub = nh.subscribe<roboteq_interface::encoder_data>("roboteq/drivemotorin/left", 1, leftDriveRoboteqCallback);
    ros::Subscriber rightDriveSub = nh.subscribe<roboteq_interface::encoder_data>("roboteq/drivemotorin/right", 1, rightDriveRoboteqCallback);
    ros::Subscriber nb1Sub = nh.subscribe<comm::nb1_to_i7_msg>("comm/nb1in/nb1in", 1, nb1Callback);
    ros::Subscriber nb2Sub = nh.subscribe<comm::nb2_3_to_i7_msg>("comm/nb2in/nb2in", 1, nb2Callback);
    ros::Subscriber nb3Sub = nh.subscribe<comm::nb2_3_to_i7_msg>("comm/nb3in/nb3in", 1, nb3Callback);
    ros::Subscriber navFilter = nh.subscribe<navigation::NavFilterOut>("navigation/navigationfilterout/navigationfilterout", 1, navFilterCallback);
    ros::Subscriber execSub = nh.subscribe<robot_control::ExecStateMachineInfo>("control/statemachineinfo/statemachineinfo", 1, execCallback);
    ros::Subscriber misSub = nh.subscribe<robot_control::MissionPlanningInfo>("/control/missionplanninginfo/missionplanninginfo", 1, misCallback);
    ros::Subscriber visionSub = nh.subscribe<computer_vision::ComputerVisionOut>("vision/computervisionout/computervisionout", 1, cvCallback);
    
    while(ros::ok())
    {
        printf("-----------------(Some Numbers Are Rounded)-----------------\n");
        printf("Laser Scanner Current Angle: %f\n", laserServo.servo_angle_current);
        printf("Camera Servo Current  Angle: %f\n", cameraServo.servo_angle_current);
        printf("NB1: Uptime: %6.4f\t Rate_P: %1.6f\t Loop Counter: %u\n", nb1.nb_clock, nb1.rate_p, nb1.main_loop_counter);
        printf("NB2: Uptime: %6.4f\t Rate_Q: %1.6f\t Loop Counter: %u\n", nb2.nb_clock, nb2.rate_p, nb2.main_loop_counter);
        printf("NB3: Uptime: %6.4f\t Rate_R: %1.6f\t Loop Counter: %u\n", nb3.nb_clock, nb3.rate_p, nb3.main_loop_counter);
        printf(" Left Roboteq Pkt Counter: %u\n", left.counter);
        printf("Right Roboteq Pkt Counter: %u\n", right.counter);
        printf("Grabber Roboteq Pkt Cnter: %u\n", grabber.counter);
        printf(":::::::::::Navigation:::::::::::\n");
        printf("Counter: %u\tNav Status: %u\tPrev RR Update: %u\tUpdate: %u\n", nav.counter, nav.nav_status, nav.prev_rr_update, nav.update);
        printf("X Pos: %1.3f\tY Pos: %1.3f\tDistance: %1.3f\n", nav.x_position, nav.y_position, hypot(nav.x_position, nav.y_position));
        printf("Roll: %1.5f\tPitch: %1.5f\tHeading: %1.5f\n", nav.roll, nav.pitch, nav.heading);
        printf("H Heading: %1.5f\tBearing: %1.5f\tVelocity %1.5f\n", nav.human_heading, nav.bearing, nav.velocity);
        printf("P1: %f\tP2: %f\tP3: %f\n", nav.p1_offset, nav.p2_offset, nav.p3_offset);
        printf("Q1: %f\tQ2: %f\tQ3: %f\n", nav.q1_offset, nav.q2_offset, nav.q3_offset);
        printf("R1: %1.6f\tR2: %1.6f\tR3: %1.6f\n", nav.r1_offset, nav.r2_offset, nav.r3_offset);
        printf("RR100: %1.6f\tRR101: %1.6f\n", nav.rr_100, nav.rr_101);
        printf("Zenith: %1.3f\tEstimated Zenith: %1.3f\n", nav.sun_zenith, nav.estimated_zenith);
        printf("Init Roll: %1.4f\tInit Pitch: %1.4f\n", nav.roll_init, nav.pitch_init);
        printf("Init Heading: %1.4f\n", nav.heading_init);
        printf("Sun North Angle: %1.2f\tText North Angle: %1.2f\n", nav.sun_north_angle, nav.text_north_angle);
        printf("North Angle: %1.6f\n", nav.north_angle);
        printf(":::::::::::Robot Control:::::::::::\n");
        printf("Exec Counter %u\t\tMission Planning Counter %u\n", exec.counter, mis.counter);
        printf("WayPnt List Type %u\tWayPnt Index %u\n", mis.waypoint_list_type, mis.waypoint_index);
        printf(":::::::::::Computer Vision:::::::::::\n");
        printf("Vision Counter %u\n", vision.counter);
        ros::spinOnce();
		loopRate.sleep(); 
    }
    
    
};

void cameraServoCallback(const comm::Servo_status::ConstPtr& msg)
{
    cameraServo = *msg;
}

void laserServoCallback(const comm::Servo_status::ConstPtr& msg)
{
    laserServo = *msg;
}

void nb1Callback(const comm::nb1_to_i7_msg::ConstPtr& msg)
{
    nb1 = *msg;
}

void nb2Callback(const comm::nb2_3_to_i7_msg::ConstPtr& msg)
{
    nb2 = *msg;
}

void nb3Callback(const comm::nb2_3_to_i7_msg::ConstPtr& msg)
{
    nb3 = *msg;
}

void leftDriveRoboteqCallback(const roboteq_interface::encoder_data::ConstPtr& msg)
{
    left = *msg;
}

void rightDriveRoboteqCallback(const roboteq_interface::encoder_data::ConstPtr& msg)
{
    right = *msg;
}

void grabberRoboteqCallback(const roboteq_interface::GrabberFeedback::ConstPtr& msg)
{
    grabber = *msg;
}

void navFilterCallback(const navigation::NavFilterOut::ConstPtr& msg)
{
    nav = *msg;
}

void misCallback(const robot_control::MissionPlanningInfo::ConstPtr& msg)
{
    mis = *msg;
}

void execCallback(const robot_control::ExecStateMachineInfo::ConstPtr& msg)
{
    exec = *msg;
}

void cvCallback(const computer_vision::ComputerVisionOut::ConstPtr& msg)
{
    vision = *msg;
}
