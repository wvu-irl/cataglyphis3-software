#include <ros/ros.h>
#include <messages/SimInfo.h>
#include <messages/SimControl.h>

enum SIM_MANAGER_STATE_T {_startRun_, _waitingForStart_, _running_, _finished_} state = _startRun_;
void simInfoCallback(const messages::SimInfo::ConstPtr& msg);

unsigned int numSamples = 0;
double elapsedtime = 0.0;
bool nodesRunning = false;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sim_manager_node");
	ros::NodeHandle nh;
	ros::Subscriber simInfoSub = nh.subscribe<messages::SimInfo>("/simulation/simulator/siminfo",1,simInfoCallback);
	ros::Publisher simControlPub = nh.advertise<messages::SimControl>("/simulation/simcontrol/simcontrol",1);
	messages::SimControl simControlMsg;
	ros::Rate loopRate(10.0);

	simControlMsg.teleport = false;
	simControlMsg.setSimSpeed = true;
	simControlMsg.simSpeed = 50.0;
	simControlMsg.pauseSwitch = false;
	simControlMsg.collision = 0;
	simControlMsg.cvFindSample = false;
	simControlMsg.pubKeyframeList = false;
	simControlMsg.rollAngle = 0.0;
	simControlMsg.homingUpdated = true;
	simControlMsg.navStopRequest = false;

	const unsigned int endConditionNumSamples = 4;
	const double endConditionElapsedTime = 2.0*3600.0; // 2 hr
	const unsigned int endConditionNumRuns = 200;

	unsigned int numRunsCompleted = 0;
	bool continueRuns = true;
	while(continueRuns && ros::ok())
	{
		switch(state)
		{
		case _startRun_:
			ROS_INFO("start run");
			elapsedtime = 0.0;
			numSamples = 0;
			system("bash ~/cataglyphis_ws/src/linux_files/donut_smashing_sim_launch.sh");
			ros::Duration(10).sleep();
			simControlPub.publish(simControlMsg);
			continueRuns = true;
			state = _running_;
			break;
		case _running_:
			ROS_INFO_THROTTLE(3,"running, numSamples = %u, time = %f, numRuns = %u",numSamples,elapsedtime,numRunsCompleted);
			if(numSamples >= endConditionNumSamples || elapsedtime >= endConditionElapsedTime)
			{
				numRunsCompleted++;
				state = _finished_;
			}
			else state = _running_;
			break;
		case _finished_:
			ROS_INFO("run finished");
			system("rosnode kill /mission_planning_node");
			system("rosnode kill /exec_node");
			system("rosnode kill /map_manager_node");
			system("rosnode kill /safe_pathing_node");
			system("rosnode kill /simulation_node");
			system("rosnode kill /HSM_Master_Executive");
			ros::Duration(10).sleep();
			state = _startRun_;
			if(numRunsCompleted >= endConditionNumRuns) continueRuns = false;
			else continueRuns = true;
			break;
		}
		ros::spinOnce();
		loopRate.sleep();
	}
	ROS_INFO("all runs complete");

	return 0;
}

void simInfoCallback(const messages::SimInfo::ConstPtr &msg)
{
	numSamples = msg->samplesFound;
	elapsedtime = msg->elapsedTime;
}
