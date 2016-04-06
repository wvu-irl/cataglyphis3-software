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

//#include <messages/NavFilterOut.h>

using namespace PointMatcherSupport;

class Mapper
{
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;

	ros::NodeHandle node;

	ros::Subscriber keycloudSub;
//	ros::Subscriber keypositionSub;		//get x, y in the map
//	ros::Subscriber transformation_IMU_Sub;		//guess transformation
	ros::Publisher mapPub;



	PM::DataPointsFilters inputFilters;

	PM::DataPoints keyPointCloud;
	PM::DataPoints refPointCloud;
	PM::DataPoints readPointCloud;

	PM::ICP icp;

	std::unique_ptr<PM::Transformation> transformation;	//use for calculate points pose in map coordinate

	int inputQueueSize; //limit the input data 
	double minOverlap;
	int minReadingPointCount;
	int minMapPointCount;
	
	std::string ref_frame;
	std::string read_frame;


	PM::TransformationParameters TreadToref;
	PM::TransformationParameters TreadTomap;
	PM::TransformationParameters TrefTomap;
	PM::TransformationParameters guessT;

//	ros::Time publishStamp;

//	tf::TransformListener tfListener;

	float x0, x1, y0, y1, heading0, heading1;
	double theta, diff_x, diff_y;

	const double PI = 3.1415926;
public:
	ros::Time mapCreationTime;
	PM::DataPoints mapPointCloud;
	std::string map_frame;
	ros::Time cloud_stamp;
	float x;
	float y;
	float heading;

	void set_parameters();
	void set_IMU_data(float IMU_x, float IMU_y, float IMU_heading);
	Mapper();

protected:
	void getMappercallback(const sensor_msgs::PointCloud2& cloudMsgIn);

	void processCloud(PM::DataPoints newPointCloud, const std::string& read_frame, const ros::Time& stamp);


};