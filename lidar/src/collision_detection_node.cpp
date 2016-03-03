#include <ros/ros.h>
#include <iostream>
#include <lidar/CollisionOut.h>

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

//subscribe to state machine info
class RobotControlSubscriber
{
private:
	ros::Subscriber sub_sm;
	ros::NodeHandle node;

	// void getExecCallback(const robot_control::ExecStateMachineInfo::ConstPtr &msg)
	// {
	// 	if(msg->var == 1) //logic to decide to run collision detection or not
	// 	{
	// 		this->check_for_collisions=1;
	// 	}
	// 	else
	// 	{
	// 		this->check_for_collisions=0;
	// 	}
	// }
public:
	int check_for_collisions;
	RobotControlSubscriber()
	{
		// sub_sm = node.subscribe("/control/execinfo/execinfo", 1, &RobotControlSubscriber::getExecCallback, this);
		check_for_collisions = 1; //change to 0 once subscribed to robot control
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "collision_detection_node");
	ROS_INFO("collision_detection_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);
	ros::Publisher pub_col = nh.advertise<lidar::CollisionOut>("lidar/collisiondetectionout/collisiondetectionout",1);

	//subscribers
	VelodyneSubscriber velodyne;
	RobotControlSubscriber robot_control;

	//output messages
	lidar::CollisionOut msg_CollisionOut;

	while(ros::ok())
	{
		//check for collisions
		if(robot_control.check_for_collisions==1) 
		{
			//function trim laser down to only points in virtual corridor

			//do ground removal on points in virtual corridor

			//check for collision

		}
		else
		{
			//don't check for collision
		}

		//populate message
		msg_CollisionOut.collision = rand(); //put result here
		msg_CollisionOut.distance_to_collision = rand(); //put result here

		//publish message
		pub_col.publish(msg_CollisionOut);

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}