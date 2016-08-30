#ifndef LIDAR_FILTERING_H
#define LIDAR_FILTERING_H
#include "ros/ros.h"
#include "ros/console.h"
#include <array>
#include <iostream>
#include <stdio.h>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
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

#include <messages/ExecInfo.h>
#include <messages/MissionPlanningInfo.h>

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

	ros::Subscriber _sub_execinfo;
	ros::Subscriber _sub_missionplanning;

	ros::Subscriber _sub_velodyne;
	void doMathMapping();
	// void doMathHoming();
	void doLongDistanceHoming();
	void setPreviousCounters();
	bool newPointCloudAvailable();
	void packLocalMapMessage(messages::LocalMap &msg);
	void packHomingMessage(messages::LidarFilterOut &msg);
	void stackClouds();
	void fitCylinderLong();
	// void fitCylinderShort();
	short int _navigation_filter_counter;
	short int _navigation_filter_counter_prev;
	short int _registration_counter;
	short int _registration_counter_prev;
	short int _stack_counter;

	float _homing_x = 0;
	float _homing_y = 0;
	float _homing_heading = 0;
	bool _homing_found = 0;
	bool _do_homing;
	float _dull_x = 0.0;
	float _dull_y = 0.0;
	float _shiny_x = 0.0;
	float _shiny_y = 0.0;
	float _cylinder_std = 0.0;

private:
	//navigation filter callback
	float _navigation_filter_x;
	float _navigation_filter_y;
	float _navigation_filter_roll;
	float _navigation_filter_pitch;
	float _navigation_filter_heading;
	bool _homing_updated_flag;
	//short int _navigation_filter_counter;
	//short int _navigation_filter_counter_prev;

	//ExecInfo callback
	bool _execinfo_turnflag;
	bool _execinfo_stopflag;

	//Mission Planning Inf callback
	bool _mission_startSLAM;

	//transform points from lidar frame to robot body
	Eigen::Matrix3f _R_tilt_robot_to_beacon; //robot to homing beacon rotation (pitch and roll rotation only)
	Eigen::Matrix3f _R_lidar_to_robot; //lidar body from to robot body frame (rotation)

	//registration callback
	pcl::PointCloud<pcl::PointXYZI> _input_cloud;
	//short int _registration_counter;
	//short int _registration_counter_prev;
	bool _registration_new;

	//stitch clouds function
	pcl::PointCloud<pcl::PointXYZI> _piece_one;
	pcl::PointCloud<pcl::PointXYZI> _piece_two;
	pcl::PointCloud<pcl::PointXYZI> _piece_three;
	pcl::PointCloud<pcl::PointXYZI> _piece_four;
	pcl::PointCloud<pcl::PointXYZI> _piece_five;

	//mapping function
	const int map_range = 60; //
	const float grid_size = 1; // size of the local map grid
	const float threshold_tree_height = 2.0; // above which the points will be disgarded
	std::vector<std::vector<float> > _local_grid_map; // local grid map without grond adjacent infomation
	std::vector<std::vector<float> > _local_grid_map_new; // local grid map with grond adjacent infomation
	pcl::PointCloud<pcl::PointXYZI> _object_filtered;

	//homing function
	struct cylinder
	{
		arma::mat points;
		arma::mat point_in_space = arma::zeros<arma::mat>(3,1);
		arma::mat axis_direction = arma::zeros<arma::mat>(3,1);
		arma::mat raius_estimate = arma::zeros<arma::mat>(1,1);
	};
	std::vector<cylinder> _cylinders;
	std::vector<cylinder> _potential_cylinders_nonintensity;
	std::vector<cylinder> _potential_cylinders_intensity;

	const float home_detection_range = 60.0;

	// //use for grid map
    typedef struct Cell 
	{
	    float x_mean;
	    float y_mean;
	    float z_mean;
	    float var_z;
	    bool ground_adjacent;
	    bool occupy; //set to 1 if that cell has been occupied
	    bool drivability; //set to 1 if that cell is drivable
    }Cell;

    //this is the grid map that store the points' x, y, z, z variance, occupy, adjacent info
    //Cell _GridMap [2*map_range/grid_size][2*map_range/grid_size];
    Cell _GridMap [500][500];

    //for the visualizer
    int visualizerCounter = 0;
    int spintime = 1000;
    // pcl::visualization::PCLVisualizer viewer;
    //pcl::PointCloud<pcl::PointXYZI> _object_filtered_projection_display;

	//callback functions
	void navigationFilterCallback(const messages::NavFilterOut::ConstPtr &msg);
	void registrationCallback(pcl::PointCloud<pcl::PointXYZI> const &input_cloud);

	void execinforCallback(const messages::ExecInfo::ConstPtr &exec_msg);
	void missionplanninginforCallback(const messages::MissionPlanningInfo::ConstPtr &mission_msg);
};

#endif // LIDAR_FILTERING_H
