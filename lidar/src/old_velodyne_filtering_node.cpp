#include <ros/ros.h>
#include <iostream>

using namespace std;

//subscribe to velodyne point cloud topic
class VelodyneSubscriber
{
private:
	ros::Subscriber sub_pc;
	ros::NodeHandle node;

	// void getPCCallback(const robot_control::ExecStateMachineInfo::ConstPtr &msg)
	// {
	// }
public:
	// VelodyneSubscriber()
	// {
	// 	sub_pc = node.subscribe("/point_cloud", 1, &VelodyneSubscriber::getPointCloudCallback, this);
	// }
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "velodyne_filtering_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	ROS_INFO("velodyne_filtering_node running...");

	VelodyneSubscriber velodyne;

	while(ros::ok())
	{

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}