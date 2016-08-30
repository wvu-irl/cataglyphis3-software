#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H
#include <robot_control/mission_planning_types_defines.h>
#include "ros/ros.h"
#include "ros/console.h"
#include <iostream>
#include <stdio.h>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <vector>
#include <cmath>
#include <lidar/patch.hpp>
#include <sstream>
#include <stdlib.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <cstdlib>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <math.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl-1.7/pcl/segmentation/approximate_progressive_morphological_filter.h> //added because of error ApproximateProgressiveMorphologicalFilter not a member of pcl
#include <armadillo>
#include <messages/CollisionOut.h>
#include <messages/CreateROIHazardMap.h>
#include <messages/NextWaypointOut.h>
#include <messages/RobotPose.h>
#include <messages/MissionPlanningInfo.h>
#include <messages/NavFilterOut.h>	//NAV
// #include <messages/ZedCollisionOut.h>
#include "pcl/conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/common/transforms.h>


class CollisionDetection
{
public:
	// Methods
	CollisionDetection();
	// Members
	ros::NodeHandle _nh;
	ros::Subscriber _sub_velodyne;
	ros::Subscriber _sub_waypoint;
	ros::Subscriber _sub_position;
	ros::Subscriber _sub_zedcollision;
	ros::Subscriber _sub_mission;

	ros::Subscriber _sub_navigation;
	// ros::ServerService ;
	void setPreviousCounters();
	bool newPointCloudAvailable();
	void packCollisionMessage(messages::CollisionOut &msg);
	int doMathSafeEnvelope();
	int doMathRANSAC();
	void Initializations();
	// bool doPredictiveAovidance();

private:
	//transform points from lidar frame to robot body
	Eigen::Matrix3f _R_lidar_to_robot; //lidar body from to robot body frame (rotation)
	Eigen::Matrix3f _R_tilt_robot_to_beacon; //robot to homing beacon rotation (pitch and roll rotation only)	//NAV

	//registration callback
	pcl::PointCloud<pcl::PointXYZI> _input_cloud;
	short int _registration_counter;
	short int _registration_counter_prev;
	//bool _registration_new;

	//safe envelope
	const float _CORRIDOR_WIDTH = 2.0; //width of virtual corridor (meters)
	const float _CORRIDOR_LENGTH_SLOWDOWN = 5.0; //length of virtual corridor (meters) slowdown the robot
	const float _CORRIDOR_LENGTH_AVOID = 3.5;
	const float _LIDAR_HEIGHT = 0.76; // height of lidar from ground (meters)
	const float _SAFE_ENVELOPE_ANGLE = 15.0*3.14159265/180.0; //safe envelope angle (radians)
	const short int _TRIGGER_POINT_THRESHOLD = 10; //number of points in safe envelope that will trigger the avoidance

	// const double PI = 3.141592653589793238462643383279502884197169399375105820974944592307816406286;

	//parameters
	double short_distance;
	double long_distance;
	double distance_second;
	double threshold_obstacle_distance;
	int threshold_obstacle_number;
	double threshold_min_angle;

	int threshold_counter_lidar;
	// int threshold_counter_zed;
	int threshold_counter_ransac;
	int threshold_counter_ransac_avoid;

	double error_angle;

	//waypoints
	double _xg;
	double _yg;

	//position
	double _xposition;
	double _yposition;
	double _headingposition;

	//zed data
	// int _zedcollision;

	//mission planning info
	bool _doingApproach;
	bool _doingExamine;
	int _currentROI;

	//counters
	int _collision_counter_lidar_slowdown;
	int _collision_counter_lidar_avoid;
	// int _collision_counter_zed;
	int _collision_counter_ransac;
	int _collision_counter_ransac_slowdown;
	int _collision_counter_ransac_avoid;

	bool _collision_counter_ransac_switch;

	//navigation filter callback 	//NAV
	float _navigation_filter_x;
	float _navigation_filter_y;
	float _navigation_filter_roll;
	float _navigation_filter_pitch;
	float _navigation_filter_heading;


	std::vector<float> _hazard_x;
	std::vector<float> _hazard_y;

	//preductive avoidance
	ros::ServiceServer returnHazardMapServ;
	std::vector<float> _hazard_map_x;
	std::vector<float> _hazard_map_y;

	bool returnHazardMap(messages::CreateROIHazardMap::Request &req, messages::CreateROIHazardMap::Response &res);

	//collision output
	short int _collision_status;
	double _distance_to_drive;
	double _angle_to_drive;
	bool _slowdown;

	void registrationCallback(pcl::PointCloud<pcl::PointXYZI> const &input_cloud);
	void waypointsCallback(messages::NextWaypointOut const &waypoint_msg);
	void positionCallback(messages::RobotPose const &position_msg);
	// void zedcollisionCallback(messages::ZedCollisionOut const &zedcollisionout_msg);
	void missionCallback(messages::MissionPlanningInfo const &msg);

	void navigationFilterCallback(const messages::NavFilterOut::ConstPtr &navigation_msg);	//NAV

	int firstChoice(double angle, double distance);
	int secondChoice(double angle, double distance, double xg, double yg);
	int finalChoice(double left_angle, double right_angle, int collision_left_counter, int collision_right_counter, double xg_local, double yg_local);
	void generateAvoidancemap();
	void generateHazardmap();
};

#endif // COLLISION_DETECTION_H






