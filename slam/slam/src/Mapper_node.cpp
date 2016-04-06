#include "slam/keyframe_Position.h"
#include "slam/Mapper.h"
#include "sensor_msgs/PointCloud2.h"
int main(int argc, char **argv)
{
	typedef PointMatcher<float> PM;


	ros::init(argc,argv, "Mapper_node");
	ros::NodeHandle node;
	ros::Rate loop_rate(5);

	keyframe_Position keyframe_position;
	Mapper mapper;

//	ros::Publisher mapPub = node.advertise<sensor_msgs::PointCloud2>("map_point", 2, true);

	//new message counter
	int prev_counter = keyframe_position.counter;

	//set parameters
	mapper.set_parameters();
	
	while(ros::ok())
	{
		if(prev_counter != keyframe_position.counter)
		{
			mapper.set_IMU_data(keyframe_position.x, keyframe_position.y, keyframe_position.heading);
		}

		prev_counter = keyframe_position.counter;
//		ROS_INFO_STREAM("point numbers: " << mapper.mapPointCloud.features.cols());
//		if(mapper.mapPointCloud.features.cols() != 0)
//		{	
//			mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapper.mapPointCloud, mapper.map_frame, mapper.cloud_stamp));
//		}

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}