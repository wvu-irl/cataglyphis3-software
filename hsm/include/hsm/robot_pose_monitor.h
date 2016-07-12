#ifndef ROBOT_POSE_MONITOR_H
#define ROBOT_POSE_MONITOR_H
#include <ros/ros.h>
#include <messages/RobotPose.h>
#include <messages/NavFilterOut.h>
#include <messages/SLAMPoseOut.h>
#include <messages/HSMSetNorthAngle.h>

class RobotPoseMonitor
{
public:
	// members
	ros::NodeHandle nh;
	ros::Publisher bestPosePub;
	ros::Subscriber navSub;
	ros::Subscriber slamSub;
	ros::ServiceServer setNorthAngleServ;
	messages::RobotPose bestPoseMsg;
	messages::NavFilterOut navMsg;
	messages::SLAMPoseOut slamMsg;
	ros::Timer poseMonitorTimer;
	const float poseMonitorPeriod = 0.05;
	float navFilterConf;
	float slamConf;
	float northAngle;
	// methods
	RobotPoseMonitor();
	void serviceMonitor(const ros::TimerEvent&);
	void navCallback(const messages::NavFilterOut::ConstPtr& msg);
	void slamCallback(const messages::SLAMPoseOut::ConstPtr& msg);
	bool setNorthAngleCallback(messages::HSMSetNorthAngle::Request& req, messages::HSMSetNorthAngle::Response& res);
};

#endif // ROBOT_POSE_MONITOR_H
