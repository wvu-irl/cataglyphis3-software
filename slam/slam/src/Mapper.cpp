#include <slam/Mapper.h>

Mapper::Mapper():
	transformation(PM::get().REG(Transformation).create("RigidTransformation"))
{
	//topics initialization
	keycloudSub = node.subscribe("current_scan", inputQueueSize, &Mapper::getMappercallback, this);
	mapPub = node.advertise<sensor_msgs::PointCloud2>("map_point", 2, true);
}

void Mapper::set_parameters()
{
	
//	inputQueueSize(getParam<int>("inputQueueSize", 10));	
	int inputQueueSize = 10; //limit the number of input data
//	minOverlap(getParam<double>("minOverlap", 0.5));
	double minOverlap = 0.5;
//	minReadingPointCount(getParam<int>("minReadingPointCount", 2000));
	int minReadingPointCount = 100;
//	minMapPointCount(getParam<int>("minMapPointCount", 500));
	int minMapPointCount = 100;
//	ref_frame(getParam<std::string>("ref_frame", "reference"));
	std::string ref_frame = "reference";
//	read_frame(getParam<std::string>("read_frame", "reading"));	
	std::string read_frame = "reading";
//	map_frame(getParam<std::string>("map_frame", "map"));
	std::string map_frame = "map";

//	publishStamp(ros::Time::now());
//	tfListener(ros::Duration(30))

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

void Mapper::set_IMU_data(float IMU_x, float IMU_y, float IMU_heading)
{
	x = IMU_x;
	y = IMU_y;
	heading = IMU_heading;
}

void Mapper::getMappercallback(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	PM::DataPoints cloud(DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));
	processCloud(cloud, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp);	//pass position to the function
	cloud_stamp = cloudMsgIn.header.stamp;
}

void Mapper::processCloud(PM::DataPoints newPointCloud, const std::string& read_frame, const ros::Time& stamp)
{
	// IMPORTANT: We need to receive the point clouds in local coordinates (reading, robot position)
	timer t;

	//Convert point cloud
	const size_t goodCount(newPointCloud.features.cols());
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}

	//Dimension of the point cloud, we seem like handle 3D
	const int dimp1(newPointCloud.features.rows());
	ROS_INFO_STREAM("Processing new point cloud");
	{
		timer t; //print how long take the algorithm

		//apply filters to incoming cloud, in robot coordinates
		inputFilters.apply(newPointCloud);

		ROS_INFO_STREAM("Input filters took "<< t.elapsed() <<" [s]");
	}


	//if it is the start point
	if (refPointCloud.features.cols() == 0)
	{
		refPointCloud = newPointCloud;
		readPointCloud = newPointCloud;
		TreadToref = PM::TransformationParameters::Identity(dimp1,dimp1);
		TreadTomap = PM::TransformationParameters::Identity(dimp1,dimp1);
		TrefTomap = PM::TransformationParameters::Identity(dimp1,dimp1);
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
	 	
	 	x1 = x;
	 	y1 = y;
	 	heading1 = heading;

	 	theta = heading1 - heading0;
	 	// transfer from degree to radians
	 	theta = theta * PI / 180;

	 	diff_x = x1 - x0;
	 	diff_y = y1 - y0;
	 	// calculate guess transformation from IMU
	 	guessT(0,0) = cos(theta);
	 	guessT(0,1) = -sin(theta);
	 	guessT(0,2) = 0;
	 	guessT(0,3) = diff_x;
	 	guessT(1,0) = sin(theta);
	 	guessT(1,1) = cos(theta);
	 	guessT(1,2) = 0;
	 	guessT(1,3) = diff_y;
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
//		TreadToref = icp(*readPointCloud, *refPointCloud, IMU_Transformation);

		//only use reading and reference to get transformation

		TreadToref = icp(readPointCloud, refPointCloud,guessT);
		ROS_DEBUG_STREAM("TreadToref (icp) :\n" << TreadToref);

		//Ensure minimum overlap between scans
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
		ROS_INFO_STREAM("Overlap: " << estimatedOverlap);
		if (estimatedOverlap < minOverlap)
		{
			ROS_ERROR_STREAM("Estimated overlap too small, ignoring ICP correction!");
			return;
		}

		//compute tf
		TreadTomap = TreadToref * TrefTomap;

		//publish map
		ROS_INFO_STREAM("Adding new points to the map");

		mapPointCloud = transformation->compute(readPointCloud, TreadTomap);

		mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapPointCloud, map_frame, cloud_stamp));



	}
	catch (PM::ConvergenceError error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return;
	}

}