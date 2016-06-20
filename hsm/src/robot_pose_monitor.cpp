#include <robot_pose_monitor.h>

RobotPoseMonitor::RobotPoseMonitor()
{
	bestPosePub = nh.advertise<messages::RobotPose>("/hsm/masterexec/globalpose", 1);
	navSub = nh.subscribe<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout", 1, &RobotPoseMonitor::navCallback, this);
	slamSub = nh.subscribe<messages::SLAMPoseOut>("/slam/keyframesnode/slamposeout", 1, &RobotPoseMonitor::slamCallback, this);
	poseMonitorTimer = nh.createTimer(ros::Duration(poseMonitorPeriod), &RobotPoseMonitor::serviceMonitor, this);
	// Trust dead reckoning right at beginning before any keyframes have been generated
	navFilterConf = 1.0;
	slamConf = 0.9;
}

void RobotPoseMonitor::serviceMonitor(const ros::TimerEvent&)
{
	// compute confidence in nav filter solution
	navFilterConf = 0.0;
	// compute confidence in slam solution
	slamConf = 1.0;
	// compare confidences in soltions and choose the one with the higher confidence
	if(slamConf > navFilterConf)
	{
		bestPoseMsg.x = slamMsg.globalX;
		bestPoseMsg.y = slamMsg.globalY;
		bestPoseMsg.heading = slamMsg.globalHeading;
		bestPoseMsg.humanHeading = fmod(bestPoseMsg.heading, 360.0);
	}
	else
	{
		bestPoseMsg.x = navMsg.x_position;
		bestPoseMsg.y = navMsg.y_position;
		bestPoseMsg.heading = navMsg.heading;
		bestPoseMsg.humanHeading = fmod(bestPoseMsg.heading, 360.0);
	}
	bestPosePub.publish(bestPoseMsg);
}

void RobotPoseMonitor::navCallback(const messages::NavFilterOut::ConstPtr &msg)
{
	navMsg = *msg;
}

void RobotPoseMonitor::slamCallback(const messages::SLAMPoseOut::ConstPtr &msg)
{
	slamMsg = *msg;
}