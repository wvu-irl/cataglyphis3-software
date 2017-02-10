#include <ros/ros.h>
#include <messages/SimInfo.h>

enum SIM_MANAGER_STATE_T {_startRun_, _running_, _finished_} state = _startRun_;
void simInfoCallback(const messages::SimInfo::ConstPtr& msg);

unsigned int numSamples = 0;
double elapsedtime = 0.0;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sim_manager_node");
	ros::NodeHandle nh;
	ros::Subscriber simInfoSub = nh.subscribe<messages::SimInfo>("/simulation/simulation/siminfo",1,simInfoCallback);
	ros::Rate loopRate(5.0);

	const unsigned int endConditionNumSamples = 4;
	const double endConditionElapsedTime = 2.0*3600.0; // 2 hr
	const unsigned int numRuns = 100;

	bool continueRuns = true;
	while(continueRuns)
	{
		switch(state)
		{
		case _startRun_:
			elapsedtime = 0.0;
			numSamples = 0.0;
			system("bash ~/cataglyphis_ws/src/linux_files/donut_smashing_sim_launch.sh");
			state = _running_;
			continueRuns = true;
			break;
		case _running_:

			break;
		}
	}

	return 0;
}

void simInfoCallback(const messages::SimInfo::ConstPtr &msg)
{
	numSamples = msg->samplesFound;
	elapsedtime = msg->elapsedTime;
}
