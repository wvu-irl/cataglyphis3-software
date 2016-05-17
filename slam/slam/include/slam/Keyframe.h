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
#include "messages/NavFilterOut.h"
#include "messages/SLAMPoseOut.h"
#include "messages/LocalMap.h"

// #include "tf/transform_broadcaster.h"
// #include "tf_conversions/tf_eigen.h"
// #include "tf/transform_listener.h"
#include "eigen_conversions/eigen_msg.h"

#include "Eigen/Dense"

#include "pcl_ros/point_cloud.h"
#include "pcl/conversions.h"
#include "sensor_msgs/PointCloud2.h"
//using namespace std;
using namespace PointMatcherSupport;

class Keyframe
{
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;

	ros::NodeHandle node;

	// data from processed point cloud
	ros::Subscriber localmapSub;
//	ros::Subscriber subscriber_nav;
//	ros::Subscriber IMU 	//position from IMU
	// key frame position and point cloud
	ros::Publisher keyframe_odomPub;
	ros::Publisher keyframe_cloudPub;
	ros::Publisher keyframe_positionPub;
	ros::Publisher frame_positionPub;

//	PM::DataPointsFilters inputFilters;

	PM::DataPoints refPointCloud;
	


	PM::ICP icp;

	int inputQueueSize; //limit the input data 
	double minOverlap;



	
	std::string ref_frame;
	
	


	PM::TransformationParameters TreadToref;
	
	PM::TransformationParameters TrefTomap;
	PM::TransformationParameters guessT;

	ros::Time publishStamp;

//	tf::TransformListener tfListener;
//	tf::TransformBroadcaster tfBroadcaster;
	float x0, x1, y0, y1, heading0, heading1;
	double theta, diff_x, diff_y, sin_theta, cos_theta;
	const double PI = 3.1415926;

public:
	PM::DataPoints keyPointCloud;
	PM::DataPoints readPointCloud;
	std::string key_frame;
	std::string read_frame;
	std::string map_frame;
	PM::TransformationParameters TkeyToprekey;
	PM::TransformationParameters TreadTomap;
	ros::Time cloud_stamp;
	float x;
	float y;
	float heading;
	double maxdistance_key;
	double distance;
	// int counter;
//	slam::transformation_msg keyframe_position;
	messages::SLAMPoseOut slamPoseOut;

	void set_parameters();

	Keyframe();




protected:
	void getlocalmapcallback(const messages::LocalMap& LocalMapMsgIn); //change the message type

	void processCloud(PM::DataPoints newPointCloud, const std::string& read_frame, const ros::Time& stamp);	
	//navigation callback for deadreckoning position
	void set_IMU_data(float IMU_x, float IMU_y, float IMU_heading);


};