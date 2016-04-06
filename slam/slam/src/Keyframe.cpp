#include "slam/Keyframe.h"


Keyframe::Keyframe()
{
	//topics initialization
	cloudSub = node.subscribe("cloud_pcd", inputQueueSize, &Keyframe::gotKeyframecallback, this);

//	positionPub = node.advertise<nav_msgs::Odometry>("pose_to_map", 50, true);
//	cloudPub = node.advertise<sensor_msgs::PointCloud2>("current_scan", 2, true);

}

void Keyframe::set_parameters()
{
	//	inputQueueSize(getParam<int>("inputQueueSize", 10));
	inputQueueSize = 1;	//limit the number of input data
//	minOverlap(getParam<double>("minOverlap", 0.5));
	minOverlap = 0.5;
//	mindistance_key(getParam<double>("mindistance_key", 1));	
	mindistance_key = 10;//min distance between each key frame
//	minReadingPointCount(getParam<int>("minReadingPointCount", 2000));
	minReadingPointCount = 2000;
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
}

void Keyframe::set_IMU_data(float IMU_x, float IMU_y, float IMU_heading)
{
	x = IMU_x;
	y = IMU_y;
	heading = IMU_heading;
}

void Keyframe::gotKeyframecallback(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	PM::DataPoints cloud(DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));
	processCloud(cloud, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp);
	cloud_stamp = cloudMsgIn.header.stamp;
}

void Keyframe::processCloud(PM::DataPoints newPointCloud, const std::string& read_frame, const ros::Time& stamp)
{
	// IMPORTANT: We need to receive the point clouds in local coordinates (reading, robot position)
	timer t;

	//Convert point cloud
	const size_t goodCount(newPointCloud.features.cols());
	ROS_DEBUG_STREAM("goodCount: "<<goodCount);
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}

	//Dimension of the point cloud, we seem like handle 3D
	const int dimp1(newPointCloud.features.rows());
	ROS_DEBUG_STREAM("dimp1: "<<dimp1);
//	ROS_INFO_STREAM("Processing new point cloud");
	{
		timer t; //print how long take the algorithm

		//apply filters to incoming cloud, in robot coordinates
		inputFilters.apply(newPointCloud);

//		ROS_INFO_STREAM("Input filters took "<< t.elapsed() <<" [s]");

	}

	//if it is the start point
	if (refPointCloud.features.cols() == 0)
	{
	ROS_DEBUG_STREAM(refPointCloud.features.cols());

		refPointCloud = newPointCloud;
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
	 	refPointCloud = readPointCloud;
	 	readPointCloud = newPointCloud;
		ROS_DEBUG_STREAM(refPointCloud.features.cols());
		ROS_DEBUG_STREAM(readPointCloud.features.cols());

		x1 = x;
	 	y1 = y;
	 	heading1 = heading;

	 	theta = heading1 - heading0;
	 	// transfer from degree to radians
	 	theta = theta * PI / 180;
	 	diff_x = x1 - x0;
	 	diff_y = y1 - y0;

	 	sin_theta = sin(theta);
	 	cos_theta = cos(theta);
	 	// calculate guess transformation from IMU
	 	guessT(0,0) = (float)cos_theta;
	 	guessT(0,1) = (float)(-sin_theta);
	 	guessT(0,2) = 0;
	 	guessT(0,3) = (float)diff_x;
	 	guessT(1,0) = (float)(sin_theta);
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

	 	x0 = x1;
	 	y0 = y1;
	 	heading0 = heading1;
	 }
	
	//use icp to calculate transformation between reading and reference
	try
	{
		//transform reference to map
		TrefTomap = TreadTomap;

		//based on IMU transformation to be a guess
		TreadToref = icp(readPointCloud, refPointCloud, guessT);

		//use reading and reference to get transformation
//		TreadToref = icp(readPointCloud, refPointCloud);
//
//		ROS_INFO_STREAM("TreadToref (icp) :\n" << TreadToref);
//		ROS_INFO_STREAM("guessT: \n" << guessT << "x= " << x);
		//Ensure minimum overlap between scans
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
//		ROS_INFO_STREAM("Overlap: " << estimatedOverlap);
		if (estimatedOverlap < minOverlap)
		{
			ROS_ERROR_STREAM("Estimated overlap too small, ignoring ICP correction!");
			return;
		}

		//compute tf
		TreadTomap = TreadToref * TrefTomap;
		ROS_DEBUG_STREAM("TreadTomap:\n"<<TreadTomap);
		//publish tf

//	put into the main function
//		tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TreadTomap, map_frame, read_frame, stamp));
		ROS_DEBUG_STREAM("TreadTomap:\n" << TreadTomap);

		//get the distance between each key frame
		ROS_DEBUG_STREAM("TkeyToprekey:\n"<<TkeyToprekey);
		TkeyToprekey = TkeyToprekey * TreadToref;
		ROS_DEBUG_STREAM("TkeyToprekey:\n"<<TkeyToprekey);
		distance = sqrt(TkeyToprekey(0,3)*TkeyToprekey(0,3) + TkeyToprekey(1,3)*TkeyToprekey(1,3) + TkeyToprekey(2,3)*TkeyToprekey(2,3));
		ROS_INFO_STREAM("distance: "<<distance);

//		if (distance >= mindistance_key)
//		{
//			keyPointCloud = readPointCloud;
//			key_frame = read_frame;
//			TkeyToprekey = PM::TransformationParameters::Identity(dimp1,dimp1);	
// is it necessary to publish position?
//			positionPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(TreadTomap,map_frame,stamp));
//			cloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(keyPointCloud, key_frame, stamp));

			//add message x y theta here, publish them, from IMU
//		}
		


	}
	catch (PM::ConvergenceError error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return;
	}


}