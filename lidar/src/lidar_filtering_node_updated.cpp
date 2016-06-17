#include <lidar/lidar_filtering.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_filtering_node");
	ROS_INFO("lidar_filtering_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(20);
	ros::Publisher pub_lidar = nh.advertise<messages::LidarFilterOut>("lidar/lidarfilteringout/lidarfilteringout",1);
	ros::Publisher pub_local_map = nh.advertise<messages::LocalMap>("/lidar/lidarfilteringnode/localmap",1);
	LidarFilter lidar_filter;
	messages::LidarFilterOut msg_LidarFilterOut;
	messages::LocalMap msg_LocalMap;
	bool low_sampling_freq = false; //set to true if we are using 5 Hz sampling frequency

	while(ros::ok())
	{
		if(lidar_filter.newPointCloudAvailable())
		{
			if (low_sampling_freq == true)
			{
				lidar_filter.stitchClouds();
				if(lidar_filter._registration_counter % 2 == 0 && lidar_filter._registration_counter != 0)
				{
					lidar_filter.doMathMapping();
					lidar_filter.doMathHoming();
					lidar_filter.packLocalMapMessage(msg_LocalMap);
					lidar_filter.packHomingMessage(msg_LidarFilterOut);
				}
			}
			else if(low_sampling_freq == false)
			{
				lidar_filter.doMathMapping();
				lidar_filter.doMathHoming();
				lidar_filter.packLocalMapMessage(msg_LocalMap);
				lidar_filter.packHomingMessage(msg_LidarFilterOut);
			}
		}

		lidar_filter.setPreviousCounters();

		pub_lidar.publish(msg_LidarFilterOut);
		pub_local_map.publish(msg_LocalMap);

		loop_rate.sleep();
		ros::spinOnce();		
	}

	return 0;
}