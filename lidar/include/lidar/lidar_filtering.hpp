#ifndef LIDAR_FILTERING_H
#define LIDAR_FILTERING_H
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
#include <messages/LidarFilterOut.h>
#include <messages/NavFilterOut.h>
#include <messages/LocalMap.h>
#include "pcl/conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/common/transforms.h>

class LidarFilter
{
public:
	// Methods
	LidarFilter();
	// Members
	ros::NodeHandle _nh;
	ros::Subscriber _sub_navigation;
	ros::Subscriber _sub_velodyne;
	void doMathMapping();
	void doMathHoming();
	void setPreviousCounters();
	bool newPointCloudAvailable();
	void packLocalMapMessage(messages::LocalMap &msg);
	void packHomingMessage(messages::LidarFilterOut &msg);
private:
	//navigation filter callback
	float _navigation_filter_x;
	float _navigation_filter_y;
	float _navigation_filter_roll;
	float _navigation_filter_pitch;
	float _navigation_filter_heading;
	short int _navigation_filter_counter;
	short int _navigation_filter_counter_prev;

	//transform points from lidar frame to robot body
	Eigen::Matrix3f _R_tilt_robot_to_beacon; //robot to homing beacon rotation (pitch and roll rotation only)
	Eigen::Matrix3f _R_lidar_to_robot; //lidar body from to robot body frame (rotation)

	//registration callback
	pcl::PointCloud<pcl::PointXYZ> _input_cloud;
	short int _registration_counter;
	short int _registration_counter_prev;

	//mapping function
	const int map_range = 40; //size of local map is 20x20 m
	const float grid_size = 1; // size of the local map grid
	const float threshold_tree_height = 10.0; // above which the points will be disgarded
	std::vector<std::vector<float> > _local_grid_map;
	pcl::PointCloud<pcl::PointXYZ> _object_filtered;

	//homing function
	struct cylinder
	{
		arma::mat points;
		arma::mat point_in_space = arma::zeros<arma::mat>(3,1);
		arma::mat axis_direction = arma::zeros<arma::mat>(3,1);
		arma::mat raius_estimate = arma::zeros<arma::mat>(1,1);
	};

	const float home_detection_range = 15.0;
	std::string fileName; //temporary for saving cylinders to file
	float _homing_x = 0;
	float _homing_y = 0;
	float _homing_heading = 0;
	bool _homing_found = 0;


	void navigationFilterCallback(const messages::NavFilterOut::ConstPtr &msg);
	void registrationCallback(pcl::PointCloud<pcl::PointXYZ> const &input_cloud);
};

#endif // LIDAR_FILTERING_H






