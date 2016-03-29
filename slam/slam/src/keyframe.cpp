// this is for generating Key frame

#include <fstream>

#include "math.h"
// using for ICP
#include "ros/ros.h"
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"
// position
#include "nav_msgs/Odometry.h"

#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "eigen_conversions/eigen_msg.h"




using namespace std;
using namespace PointMatcherSupport;

class keyframe
{
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;

	ros::NodeHandle& node;

	// data from processed point cloud
	ros::Subscriber cloudSub;
//	ros::Subscriber IMU 	//position from IMU
	// key frame position and point cloud
	ros::Publisher positionPub;
	ros::Publisher cloudPub;

	PM::DataPointsFilters inputFilters;

	PM::DataPoints refPointCloud;
	PM::DataPoints readPointCloud;
	PM::DataPoints keyPointCloud;

	PM::ICP icp;

	int inputQueueSize; //limit the input data 
	double minOverlap;
	double mindistance_key;
	int minReadingPointCount;

	
	string ref_frame;
	string read_frame;
	string map_frame;
	string key_frame;

	PM::TransformationParameters TreadToref;
	PM::TransformationParameters TreadTomap;
	PM::TransformationParameters TrefTomap;
	PM::TransformationParameters TkeyToprekey;

	ros::Time publishStamp;

	tf::TransformListener tfListener;
	tf::TransformBroadcaster tfBroadcaster;

public:
	keyframe(ros::NodeHandle& node);



protected:
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn);

	void processCloud(PM::DataPoints newPointCloud, const std::string& read_frame, const ros::Time& stamp);	

};

keyframe::keyframe(ros::NodeHandle& node):


	node(node),
	pn(pn),

	inputQueueSize(getParam<int>("inputQueueSize", 10)),	//limit the number of input data
	minOverlap(getParam<double>("minOverlap", 0.5)),
	mindistance_key(getParam<double>("mindistance_key", 10)),	//min distance between each key frame
	minReadingPointCount(getParam<int>("minReadingPointCount", 2000)),
	minMapPointCount(getParam<int>("minMapPointCount", 500)),
	ref_frame(getParam<string>("ref_frame", "reference")),
	read_frame(getParam<string>("read_frame", "reading")),	
	map_frame(getParam<string>("map_frame", "map")),
	key_frame(getParam<string>("key_frame", "keyframe")),
	// TreadToref(PM::TransformationParameters::Identity(4,4)),
	// TreadTomap(PM::TransformationParameters::Identity(4,4)),
	// TrefTomap(PM::TransformationParameters::Identity(4,4)),		
	// TkeyToprekey(PM::TransformationParameters::Identity(4,4)),
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
	cloudSub = node.subscribe("cloud_pcd", inputQueueSize, &keyframe::gotCloud, this);

	positionPub = node.advertise<nav_msgs::Odometry>("pose_to_map", 50, true);
	cloudPub = node.advertise<sensor_msgs::PointCloud2>("current_scan", 2, true);

}


void keyframe::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	PM::DataPoints cloud(DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));
	processCloud(cloud, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp);
}

void keyframe::processCloud(PM::DataPoints newPointCloud, const std::string& read_frame, const ros::Time& stamp)
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
	ROS_EDBUG_STREAM("dimp1: "<<dimp1);
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
	ROS_DEBUG_STREAM(refPointCloud.features.cols());

		refPointCloud = newPointCloud;
		readPointCloud = newPointCloud;
		TreadToref = PM::TransformationParameters::Identity(dimp1,dimp1);
		TreadTomap = PM::TransformationParameters::Identity(dimp1,dimp1);
		TrefTomap = PM::TransformationParameters::Identity(dimp1,dimp1);
		TkeyToprekey = PM::TransformationParameters::Identity(dimp1,dimp1);	
	}
	else
	{
	 	refPointCloud = readPointCloud;
	 	readPointCloud = newPointCloud;
		ROS_DEBUG_STREAM(refPointCloud.features.cols());
		ROS_DEBUG_STREAM(readPointCloud.features.cols());
	 }
	
	//use icp to calculate transformation between reading and reference
	try
	{
		//transform reference to map
		TrefTomap = TreadTomap;

		//based on IMU transformation to be a guess
//		TreadToref = icp(readPointCloud, refPointCloud, IMU_Transformation);

		//use reading and reference to get transformation
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
		ROS_DEBUG_STREAM("TreadTomap:\n"<<TreadTomap);
		//publish tf
		tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TreadTomap, map_frame, read_frame, stamp));
		ROS_DEBUG_STREAM("TreadTomap:\n" << TreadTomap);

		//get the distance between each key frame
		ROS_DEBUG_STREAM("TkeyToprekey:\n"<<TkeyToprekey);
		TkeyToprekey = TkeyToprekey * TreadToref;
		ROS_DEBUG_STREAM("TkeyToprekey:\n"<<TkeyToprekey);
		double distance = sqrt(TkeyToprekey(0,3)*TkeyToprekey(0,3) + TkeyToprekey(1,3)*TkeyToprekey(1,3) + TkeyToprekey(2,3)*TkeyToprekey(2,3));
		ROS_INFO_STREAM("distance: "<<distance);

		if (distance >= mindistance_key)
		{
			keyPointCloud = readPointCloud;
			key_frame = read_frame;
			TkeyToprekey = PM::TransformationParameters::Identity(dimp1,dimp1);	
			positionPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(TreadTomap,map_frame,stamp));
			cloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(keyPointCloud, key_frame, stamp));

			//add message x y theta here, publish them, from IMU
		}
		


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
	ros::init(argc, argv, "keyframe");
	ros::NodeHandle node;

	keyframe keyframe(node);
	ros::spin();
	return 0;
}