#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H
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
	void setPreviousCounters();
	bool newPointCloudAvailable();
	void packCollisionMessage(messages::CollisionOut &msg);
	int doMathSafeEnvelope();
	int doMathRANSAC();
private:
	//transform points from lidar frame to robot body
	Eigen::Matrix3f _R_lidar_to_robot; //lidar body from to robot body frame (rotation)

	//registration callback
	pcl::PointCloud<pcl::PointXYZI> _input_cloud;
	short int _registration_counter;
	short int _registration_counter_prev;
	bool _registration_new;

	//safe envelope
	const float _CORRIDOR_WIDTH = 1.5; //width of virtual corridor (meters)
	const float _CORRIDOR_LENGTH = 5.0; //length of virtual corridor (meters)
	const float _LIDAR_HEIGHT = 0.76; // height of lidar from ground (meters)
	const float _SAFE_ENVELOPE_ANGLE = 15.0*3.14159265/180.0; //safe envelope angle (radians)
	const short int _TRIGGER_POINT_THRESHOLD = 10; //number of points in safe envelope that will trigger the avoidance

	//collision output
	short int _collision_status;

	void registrationCallback(pcl::PointCloud<pcl::PointXYZI> const &input_cloud);
};

#endif // COLLISION_DETECTION_H






