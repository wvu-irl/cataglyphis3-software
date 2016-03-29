#include <fstream>

#include "math.h"

#include "ros/ros.h"
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

#include "nav_msgs/Odometry.h"

#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "eigen_conversions/eigen_msg.h"

using namespace std;
using namespace PointMatcherSupport;

class keymapper
{
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;

	ros::NodeHandle& node;

	ros::Subscriber keycloudSub;
//	ros::Subscriber keypositionSub;		//get x, y in the map
//	ros::Subscriber transformation_IMU_Sub;		//guess transformation
	ros::Publisher mapPub;

	ros::Time mapCreationTime;

	PM::DataPointsFilters inputFilters;

	PM::DataPoints keyPointCloud;
	PM::DataPoints refPointCloud;
	PM::DataPoints readPointCloud;
	PM::DataPoints mapPointCloud;

	PM::ICP icp;

	unique_ptr<PM::Transformation> transformation;	//use for calculate points pose in map coordinate

	int inputQueueSize; //limit the input data 
	double minOverlap;
	int minReadingPointCount;
	int minMapPointCount;
	
	string ref_frame;
	string read_frame;
	string map_frame;

	PM::TransformationParameters TreadToref;
	PM::TransformationParameters TreadTomap;
	PM::TransformationParameters TrefTomap;

	ros::Time publishStamp;

	tf::TransformListener tfListener;

public:
	keymapper(ros::NodeHandle& node);

protected:
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn);

	void processCloud(PM::DataPoints newPointCloud, const std::string& read_frame, const ros::Time& stamp);

	void setMap(PM::DataPoints newPointCloud); //publish the map points
	DP updateMap(DP newPointCloud, const PM::TransformationParameters Ticp);

};

keymapper::keymapper(ros::NodeHandle& node):


	node(node),

	transformation(PM::get().REG(Transformation).create("RigidTransformation")),
	inputQueueSize(getParam<int>("inputQueueSize", 10)),	//limit the number of input data
	minOverlap(getParam<double>("minOverlap", 0.5)),
	minReadingPointCount(getParam<int>("minReadingPointCount", 2000)),
	minMapPointCount(getParam<int>("minMapPointCount", 500)),
	ref_frame(getParam<string>("ref_frame", "reference")),
	read_frame(getParam<string>("read_frame", "reading")),	
	map_frame(getParam<string>("map_frame", "map")),

	TreadToref(PM::TransformationParameters::Identity(4,4)),
	TreadTomap(PM::TransformationParameters::Identity(4,4)),
	TrefTomap(PM::TransformationParameters::Identity(4,4)),		

	publishStamp(ros::Time::now()),	
tfListener(ros::Duration(30))

{

	//set logger
	if (getParam<bool>("useROSlogger", false))
		PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);

	//load configs
	string configFileName;
	if (ros::param::get("~icpConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
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
		ifstream ifs(configFileName.c_str());
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

	//topics initialization
	keycloudSub = node.subscribe("current_scan", inputQueueSize, &keymapper::gotCloud, this);
//	keypositionSub = node.subscribe()	//x, y , theta message
	
	mapPub = node.advertise<sensor_msgs::PointCloud2>("map_point", 2, true);


}


void keymapper::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)	//add x, y, theta message in
{
	PM::DataPoints cloud(DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));
	processCloud(cloud, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp);	//pass position to the function

}

void keymapper::processCloud(PM::DataPoints newPointCloud, const std::string& read_frame, const ros::Time& stamp)
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
		/* get original postion 
		float x0 = ;
		float y0 = ;
		float theta0 = ;
		*/
	}
	else
	{
	 	refPointCloud = readPointCloud;
	 	readPointCloud = newPointCloud;
	 	/*
	 	float x1 = ;
	 	float y1 = ;
	 	float theta1 = ;
	 	// calculate guess transformation from IMU
	 	guessT = [cos() sin() ];

	 	x0 = x1;
	 	y0 = y1;
	 	thetal0 = theta1;

	 	*/
	}
	
	//use icp to calculate transformation between reading and reference
	try
	{
		//transform reference to map
		TrefTomap = TreadTomap;

		//based on IMU transformation to be a guess
//		TreadToref = icp(*readPointCloud, *refPointCloud, IMU_Transformation);

		//only use reading and reference to get transformation

		TreadToref = icp(readPointCloud, refPointCloud,TreadToref);
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
		mapCreationTime = stamp;
		ROS_INFO_STREAM("Adding new points to the map");

		mapPointCloud = transformation->compute(readPointCloud, TreadTomap);


		mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapPointCloud, map_frame, mapCreationTime));



	}
	catch (PM::ConvergenceError error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return;
	}

	
}

//main function supporting the keyframe class
int main(int argc, char **argv)
{
	ros::init(argc, argv, "keymapper");
	ros::NodeHandle node;

	keymapper keymapper(node);
	ros::spin();
	return 0;
}