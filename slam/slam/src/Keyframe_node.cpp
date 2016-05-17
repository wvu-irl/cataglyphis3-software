#include "slam/Keyframe.h"

int main(int argc, char **argv)
{
	typedef PointMatcher<float> PM;


	ros::init(argc,argv, "Keyframe_node");
//	ros::NodeHandle node;
	Keyframe keyframe;
/*	ros::Rate loop_rate(1000);

	Position position;
	Keyframe keyframe;

	ros::Publisher keyframe_positionPub = node.advertise<slam::transformation_msg>("keyframe_position", 1, true);
	ros::Publisher keyframe_cloudPub = node.advertise<sensor_msgs::PointCloud2>("current_scan", 2, true);
	ros::Publisher keyframe_odomPub = node.advertise<nav_msgs::Odometry>("pose_to_map", 50, true);

//	tf::TransformListener tfListener;
//	tf::TransformBroadcaster tfBroadcaster;

	//output message
	slam::transformation_msg keyframe_position;

	//new message counter
	int prev_counter_position = position.counter;
	int prev_counter_keyframe = keyframe.counter;
*/	//set parameters
	keyframe.set_parameters();

//	while(ros::ok())
//	{
		/*
		if(prev_counter_position != position.counter && prev_counter_keyframe != keyframe.counter)
		{
			keyframe.set_IMU_data(position.x, position.y, position.heading);
//			ROS_INFO_STREAM("IMU_X = " << position.x << " keyfram_X = " <<keyframe.x);
//			tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(keyframe.TreadTomap, keyframe.map_frame, keyframe.read_frame, keyframe.cloud_stamp));
		}

		prev_counter_position = position.counter;
		ROS_INFO_STREAM("keyfram.distance = " << keyframe.distance << " keyframe.mindistance_key = " <<keyframe.mindistance_key);
//		ROS_INFO_STREAM(position.counter);
		if (keyframe.distance >= keyframe.mindistance_key && prev_counter_keyframe != keyframe.counter)
		{
			keyframe.keyPointCloud = keyframe.readPointCloud;
			keyframe.key_frame = keyframe.read_frame;
			keyframe.TkeyToprekey = PM::TransformationParameters::Identity(4,4);	

			keyframe_position.x = keyframe.x;
			keyframe_position.y = keyframe.y;
			keyframe_position.heading = keyframe.heading;
//			ROS_INFO_STREAM("IMU_X = " << position.counter);
			keyframe_positionPub.publish(keyframe_position);
			keyframe_cloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(keyframe.keyPointCloud, keyframe.key_frame, keyframe.cloud_stamp));
			keyframe_odomPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(keyframe.TreadTomap,keyframe.map_frame,keyframe.cloud_stamp));
		}

		prev_counter_keyframe = keyframe.counter;
		
		loop_rate.sleep();
	*/
//		ros::spinOnce();
//	}
		ros::spin();
	return 0;
}