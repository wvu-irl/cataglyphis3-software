#include <lidar/collision_detection.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "collision_filtering_node");
	ROS_INFO("collision_filtering_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(20);
	ros::Publisher pub_col = nh.advertise<messages::CollisionOut>("lidar/collisiondetectionout/collisiondetectionout",1);
	CollisionDetection collision_detection;
	messages::CollisionOut msg_CollisionOut;

	while(ros::ok())
	{
		if(collision_detection.newPointCloudAvailable())
		{
			collision_detection.doMathSafeEnvelope();
			collision_detection.packCollisionMessage(msg_CollisionOut);
		}
		collision_detection.setPreviousCounters();
		pub_col.publish(msg_CollisionOut);
		loop_rate.sleep();
		ros::spinOnce();		
	}

	return 0;
}