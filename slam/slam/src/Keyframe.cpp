// this is for generating Key frame

#include "slam/Keyframe.h"



Keyframe::Keyframe()
{
	//topics initialization
//	counter = 0;
	localmapSub = node.subscribe("/lidar/lidarfilteringnode/localmap", 1, &Keyframe::getlocalmapcallback, this);

	//keyframe_cloudPub = node.advertise<sensor_msgs::PointCloud2>("current_scan", 2, true);
	//ros::Publisher keyframe_odomPub = node.advertise<nav_msgs::Odometry>("pose_to_map", 50, true);
	frame_positionPub = node.advertise<messages::SLAMPoseOut>("/slam/keyframe/slamposeout", 1, true);

}

void Keyframe::set_parameters()
{
//	minOverlap(getParam<double>("minOverlap", 0.5));
	minOverlap = 0.5;
//	mindistance_key(getParam<double>("mindistance_key", 1));	
	maxdistance_key = 20;//min distance between each key frame
//	minMapPointCount(getParam<int>("minMapPointCount", 500));
//	minMapPointCount = 500;
//	ref_frame(getParam<string>("ref_frame", "reference"));
	ref_frame = "reference";
//	read_frame(getParam<string>("read_frame", "reading"));
	read_frame = "reading";	
//	map_frame(getParam<string>("map_frame", "map"));
	map_frame = "map";
//	key_frame(getParam<string>("key_frame", "keyframe"));
	key_frame = "keyframe";
	// TreadToref(PM::TransformationParameters::Identity(4,4)),
	// TreadTomap(PM::TransformationParameters::Identity(4,4)),
	// TrefTomap(PM::TransformationParameters::Identity(4,4)),		
	// TkeyToprekey(PM::TransformationParameters::Identity(4,4)),
//	publishStamp(ros::Time::now());
//	tfListener(ros::Duration(30));
	TreadToref = PM::TransformationParameters::Identity(4,4);
	TreadTomap = PM::TransformationParameters::Identity(4,4);
	TrefTomap = PM::TransformationParameters::Identity(4,4);
	TkeyToprekey = PM::TransformationParameters::Identity(4,4);
	guessT = PM::TransformationParameters::Identity(4,4);
	//set logger
	if (getParam<bool>("useROSlogger", false))
		PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);

	//load configs
	std::string configFileName;
	if (ros::param::get("~icpConfig", configFileName))
	{
		std::ifstream ifs(configFileName.c_str());
		if (ifs.good())
			icp.loadFromYaml(ifs);
		else
		{
			ROS_ERROR_STREAM("Cannot load ICP config from YAML file " << configFileName);
			icp.setDefault();
		}
	}
	else
	{
		ROS_INFO_STREAM("No ICP config file given, using default");
		icp.setDefault();
	}
	//input filter
	if (getParam<bool>("useROSLogger", false))
		PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);
	
	if (ros::param::get("~inputFiltersConfig", configFileName))
	{
		std::ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			inputFilters = PM::DataPointsFilters(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load input filters config from YAML file " << configFileName);
		}
	}
	else
	{
		ROS_INFO_STREAM("No input filters config file given, not using these filters");
	}

	slamPoseOut.x = 0;
	slamPoseOut.y = 0;
	slamPoseOut.heading = 0;
	
}
void Keyframe::set_IMU_data(float IMU_x, float IMU_y, float IMU_heading)
{
	x = IMU_x;
	y = IMU_y;
	heading = IMU_heading;
	ROS_INFO_STREAM("test1");
}

void Keyframe::getlocalmapcallback(const messages::LocalMap& LocalMapMsgIn)
{

	//***************************************//
	if (LocalMapMsgIn.new_data)
	{
	set_IMU_data(LocalMapMsgIn.x_filter, LocalMapMsgIn.y_filter, LocalMapMsgIn.heading_filter);
	ROS_INFO_STREAM(LocalMapMsgIn.x_filter<<LocalMapMsgIn.y_filter<<LocalMapMsgIn.heading_filter);

	// transfer from x y points to ros pointcloud2
	pcl::PointCloud<pcl::PointXYZ> PCLcloudMsgIn;
	pcl::PCLPointCloud2 tmp_cloud;
	sensor_msgs::PointCloud2 cloudMsgIn;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr PCLcloudMsgIn (new pcl::PointCloud<pcl::PointXYZ>);

	float z_test = 0;
	for (int i=0; i < LocalMapMsgIn.x_mean.size(); i++)
	{

		// ROS_INFO_STREAM("number of localmap size: "<< LocalMapMsgIn.x_mean.size());
		// ROS_INFO_STREAM("LocalMapMsgIn.x_mean "<< i <<" "<< LocalMapMsgIn.x_mean[i]);
		// ROS_INFO_STREAM("LocalMapMsgIn.y_mean "<< i <<" "<< LocalMapMsgIn.y_mean[i]);
		float z_test = 0;

		// PCLcloudMsgIn->points[i].x = LocalMapMsgIn.x_mean[i];
		// PCLcloudMsgIn->points[i].y = LocalMapMsgIn.y_mean[i];
		// PCLcloudMsgIn->points[i].z = z_test;
		// if (LocalMapMsgIn.x_mean[i] == 0 && LocalMapMsgIn.y_mean[i] == 0)
		// {
		// 	continue;
		// }
		// else
		// {
			PCLcloudMsgIn.push_back (pcl::PointXYZ(LocalMapMsgIn.x_mean[i], LocalMapMsgIn.y_mean[i],z_test));
		// 	ROS_INFO_STREAM("PCLcloudMsgIn z "<< i <<" "<< PCLcloudMsgIn.points[i].z);
		// // }
	}
	// *PCLcloudMsgIn = PCLcloudMsgIn_temp;
	

	pcl::toPCLPointCloud2(PCLcloudMsgIn,tmp_cloud);
	pcl_conversions::fromPCL(tmp_cloud, cloudMsgIn);

	ROS_INFO_STREAM("number of localmap size: "<< LocalMapMsgIn.x_mean.size());
	ROS_INFO_STREAM("cloudMsgIn: "<<cloudMsgIn.width);



	//do ICP
	PM::DataPoints cloud(DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));
	// read_frame = "robot";
	// ROS_INFO_STREAM("Time stamp "<< cloudMsgIn.header.stamp);
	// ROS_INFO_STREAM("frame_id "<< cloudMsgIn.header.frame_id);
	processCloud(cloud, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp);
	// cloud_stamp = cloudMsgIn.header.stamp;
	// counter++;
	// ROS_INFO_STREAM("counter: " <<counter);
	}
}

void Keyframe::processCloud(PM::DataPoints newPointCloud, const std::string& read_frame, const ros::Time& stamp)
{
	// IMPORTANT: We need to receive the point clouds in local coordinates (reading, robot position)

	//Convert point cloud
	const size_t goodCount(newPointCloud.features.cols());
	ROS_INFO_STREAM("goodCount: "<<goodCount);
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	
	{
		timer t; //print how long take the algorithm

		//apply filters to incoming cloud, in robot coordinates
		inputFilters.apply(newPointCloud);

		ROS_INFO_STREAM("Input filters took "<< t.elapsed() <<" [s]");

	}
	//Dimension of the point cloud, we seem like handle 3D
	const int dimp1(newPointCloud.features.rows());
	// ROS_INFO_STREAM("dimp1: "<<dimp1);
//	ROS_INFO_STREAM("Processing new point cloud");

	//if it is the start point
	if (refPointCloud.features.cols() == 0)
	{
	// ROS_INFO_STREAM(refPointCloud.features.cols());
	// ROS_INFO_STREAM(readPointCloud.features.cols());
		refPointCloud = newPointCloud;
		keyPointCloud = newPointCloud;
		readPointCloud = newPointCloud;
		TreadToref = PM::TransformationParameters::Identity(dimp1,dimp1);
		TreadTomap = PM::TransformationParameters::Identity(dimp1,dimp1);
		TrefTomap = PM::TransformationParameters::Identity(dimp1,dimp1);
		TkeyToprekey = PM::TransformationParameters::Identity(dimp1,dimp1);
		guessT = PM::TransformationParameters::Identity(dimp1,dimp1);

		// get original postion 
		x0 = x;
		y0 = y;
		heading0 = heading;	
	}
	else
	{
		if (refPointCloud.features.cols() != keyPointCloud.features.cols())
	 	{
	 	refPointCloud = keyPointCloud;
	 	x0 = x1;
	 	y0 = y1;
	 	heading0 = heading1;

	 	//transform reference to map
		TrefTomap = TreadTomap;
	 	}

	 	readPointCloud = newPointCloud;
		ROS_INFO_STREAM(refPointCloud.features.cols());
		ROS_INFO_STREAM(readPointCloud.features.cols());

		x1 = x;
	 	y1 = y;
	 	heading1 = heading;

	 	theta = heading1 - heading0;
	 	ROS_INFO_STREAM("theta in degree: " << theta);
	 	// transfer from degree to radians
	 	theta = theta * PI / 180;
	 	diff_x = x1 - x0;
	 	diff_y = y1 - y0;

	 	sin_theta = sin(theta);
	 	cos_theta = cos(theta);
	 	// calculate guess transformation from IMU
	 	guessT(0,0) = (float)cos_theta;
	 	guessT(0,1) = (float)sin_theta;
	 	guessT(0,2) = 0;
	 	guessT(0,3) = (float)diff_x;
	 	guessT(1,0) = (float)(-sin_theta);
	 	guessT(1,1) = (float)cos_theta;
	 	guessT(1,2) = 0;
	 	guessT(1,3) = (float)diff_y;
	 	guessT(2,0) = 0;
	 	guessT(2,1) = 0;
	 	guessT(2,2) = 1;
	 	guessT(2,3) = 0;
	 	guessT(3,0) = 0;
	 	guessT(3,1) = 0;
	 	guessT(3,2) = 0;
	 	guessT(3,3) = 1;
	 	ROS_INFO_STREAM("guessT: \n" << guessT);

	 }
	
	//use icp to calculate transformation between reading and reference
	try
	{

		// ROS_INFO_STREAM("test1");
		// ROS_INFO_STREAM(refPointCloud.features.cols());
		// ROS_INFO_STREAM(readPointCloud.features.cols());
		//based on IMU transformation to be a guess
		TreadToref = icp(readPointCloud, refPointCloud, guessT);

		//use reading and reference to get transformation
		// TreadToref = icp(readPointCloud, refPointCloud);
//
		ROS_INFO_STREAM("TreadToref (icp) :\n" << TreadToref);
		
		


		//Ensure minimum overlap between scan and prekeyframe
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
		ROS_INFO_STREAM("Overlap: " << estimatedOverlap);

				//get the distance between each key frame
		ROS_DEBUG_STREAM("TkeyToprekey:\n"<<TkeyToprekey);
		TkeyToprekey = TreadToref;
		ROS_DEBUG_STREAM("TkeyToprekey:\n"<<TkeyToprekey);
		distance = sqrt(TkeyToprekey(0,3)*TkeyToprekey(0,3) + TkeyToprekey(1,3)*TkeyToprekey(1,3) + TkeyToprekey(2,3)*TkeyToprekey(2,3));
		ROS_INFO_STREAM("distance: "<<distance);


		//compute tf
		TreadTomap = TreadToref * TrefTomap;
		ROS_DEBUG_STREAM("TreadTomap:\n"<<TreadTomap);
		//publish tf

		//	put into the main function
		//tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TreadTomap, map_frame, read_frame, stamp));
		ROS_INFO_STREAM("TreadTomap:\n" << TreadTomap);

		slamPoseOut.x = slamPoseOut.x + TreadTomap(0,3);
		slamPoseOut.y = slamPoseOut.y + TreadTomap(1,3);

		ROS_INFO_STREAM("TreadTomap(0,0): " << TreadTomap(0,0));
		ROS_INFO_STREAM("TreadTomap(0,0): " << (acos(TreadTomap(0,0)) * 180.0 / PI));
		//calculate the angle
		slamPoseOutheading0 = slamPoseOutheading1;

		if (TreadTomap(0,0) != 0)
			slamPoseOutheading1 = atan2(TreadTomap(0,1), TreadTomap(0,0)) * 180.0 / PI;
		else 
			slamPoseOutheading1 = 90.0;
	
		// slamPoseOut.heading = slamPoseOut.heading + (acos((double)TreadTomap(0,0)) * 180.0 / PI);
		slamPoseOut.heading = slamPoseOut.heading + (slamPoseOutheading1 - slamPoseOutheading0);

		frame_positionPub.publish(slamPoseOut);

		//*****************test result******************//
		ROS_INFO_STREAM("slamPoseOut.x: "<<slamPoseOut.x<<"\n"
						<<"slamPoseOut.y: "<<slamPoseOut.y<<"\n"
						<<"slamPoseOut.heading: "<<slamPoseOut.heading<<"\n"
						<<"NavfilterOut.x: "<<x1<<"\n"
						<<"NavfilterOut.y: "<<y1<<"\n"
						<<"NavfilterOut.heading: "<<heading1<<"\n");
		//***********************************************//
		if (estimatedOverlap < minOverlap || distance >= maxdistance_key)
		{


			keyPointCloud = readPointCloud;
			key_frame = read_frame;
			TkeyToprekey = PM::TransformationParameters::Identity(dimp1,dimp1);	
//is it necessary to publish position?
			// keyframe_position.x = x1;
			// keyframe_position.y = y1;
			// keyframe_position.heading = heading1;
			
			// keyframe_positionPub.publish(keyframe_position);
			keyframe_odomPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(TreadTomap,map_frame,stamp));
			keyframe_cloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(keyPointCloud, key_frame, stamp));
		
//			add message x y theta here, publish them, from IMU
		}

	}
	catch (PM::ConvergenceError error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return;
	}


}