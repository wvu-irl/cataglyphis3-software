#include <ros/ros.h>
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
#include <pcl/segmentation/approximate_progressive_morphological_filter.h> //added because of error ApproximateProgressiveMorphologicalFilter not a member of pcl

using namespace std;

int map_range = 40; //size of local map is 20x20 m
float grid_size = 1; // size of the local map grid
float threshold_tree_height = 10.0; // above which the points will be disgarded
float home_detection_range = 15.0;

class Registration
{
private:
	ros::NodeHandle nn;
	ros::Subscriber sub_laser;

	void registrationCallback(pcl::PointCloud<pcl::PointXYZ> const &input_cloud)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		*cloud = input_cloud;
	}
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	Registration()
	{
		sub_laser = nn.subscribe("/velodyne_points", 1, &Registration::registrationCallback, this);

	}
};

// Registration::Registration()
// {
// 	sub_laser=nn.subscribe("velodyne_points",1,&Registration::registrationCallback,this);

// }

int main(int argc, char **argv)
{
	ros::init(argc, argv, "velodyne_filtering_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	ROS_INFO("velodyne_filtering_node running...");

	Registration registration;

	while(ros::ok())
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr ground_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr object_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr object_filtered_projection (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr local_map (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr test (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointIndicesPtr ground (new pcl::PointIndices);
	    pcl::PointCloud<pcl::PointXYZ> new_point;
	    new_point.width  = (2*map_range)*(2*map_range);
	    new_point.height = 1;
	    new_point.points.resize (new_point.width * new_point.height);
	    pcl::PointCloud<pcl::PointXYZ> ground_point;
	    ground_point.width  = (2*map_range)*(2*map_range);
	    ground_point.height = 1;
	    ground_point.points.resize (ground_point.width * ground_point.height);

	    // THESE ARE FOR CYLINDER DETECTION
	    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_middle (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    	//pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());

	    // PASS THROUGH FILTER
	    pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(registration.cloud);
	    pass.setFilterFieldName("x");
	    pass.setFilterLimits(-map_range,map_range);
	    pass.filter(*registration.cloud);
	    pass.setFilterFieldName("y");
	    pass.setFilterLimits(-map_range,map_range);
	    pass.filter(*registration.cloud);
	    pass.setFilterFieldName("z");
	    pass.setFilterLimits(-1.5,threshold_tree_height);
	    pass.filter(*registration.cloud);
	    cout << "Regional cloud " << " has " << registration.cloud->points.size() << " points." << endl;

	    // CREATE THE FILTERING OBJECT
	    pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
	    pmf.setInputCloud (registration.cloud);
	    pmf.setMaxWindowSize (20);
	    pmf.setSlope (1.0f);
	    pmf.setInitialDistance (0.5f);
	    pmf.setMaxDistance (1.5f);
	    //cout << "Cell size is " << pmf.getCellSize() << endl;
	    pmf.extract (ground->indices);

	    // CREATE THE FILTERING OBJECT
	    pcl::ExtractIndices<pcl::PointXYZ> extract;
	    extract.setInputCloud (registration.cloud);
	    extract.setIndices (ground);
	    extract.filter (*ground_filtered);

	    // EXTRACT NON-GROUND RETURNS
	    extract.setNegative (true);
	    extract.filter (*object_filtered);
	    *object_filtered_projection = *object_filtered;
	    // PROJECTION, MAY NEED TO MODIFY THE ALGORITHM LATER
	    for (int i=0; i<object_filtered->points.size(); i++)
	    {
	      object_filtered_projection->points[i].z=0;
	    }

	    // BUILD LOCAL MAP, THE CURRENT ALGORITHM IS VERY UN-EFFECTIVE, NEED TO IMPROVE LATER
		vector < pcl::PointCloud<pcl::PointXYZ>::Ptr > local_grid;
		cout << "Object after projection has " << object_filtered_projection->points.size() << " points." << endl;
		for (int i=0; i<(2*map_range)*(2*map_range); i++) // generate gird element
		{
			pass.setInputCloud(object_filtered_projection);
			pass.setFilterFieldName("y");
			pass.setFilterLimits(-map_range + floor(i/(2*map_range)) , -map_range + floor((i/2*map_range)) + grid_size); //ML direction
			pass.filter(*grid_cloud);
			pass.setInputCloud(grid_cloud);
			pass.setFilterFieldName("x");
			pass.setFilterLimits(-map_range + i % (2*map_range) , -map_range + i % (2*map_range) + grid_size); //ML direction
			pass.filter(*grid_cloud);
			//cout << "grid has " << grid_cloud->points.size() << " points" << endl;
			//local_grid.push_back((*grid_cloud).make_shared());
			local_grid.push_back(grid_cloud);
			//cout << "Size of grid map " << local_grid.size() << endl;
			cout << "Grid " << i << " has " << (local_grid[i])->points.size() << " points."<< endl;
		}

		// THE local_grid HAS ALL INFO, NEED TO PUBLISH IT


		// FROM HERE, IS THE HOME BEACON CYLINDER DETECTION PART
		// ONLY KEEP POINTS WITHIN THE HOME DETECTION RANGE
		pass.setInputCloud(registration.cloud);
	    pass.setFilterFieldName("z");
	    pass.setFilterLimits(-1.5,3); 
	    pass.filter(*registration.cloud);
	    pass.setInputCloud(registration.cloud);
	    pass.setFilterFieldName("x");
	    pass.setFilterLimits(-home_detection_range,home_detection_range);
	    pass.filter(*registration.cloud);
	    pass.setInputCloud(registration.cloud);
	    pass.setFilterFieldName("y");
	    pass.setFilterLimits(-home_detection_range,home_detection_range); 
	    pass.filter(*registration.cloud);
	    cout << "PassThrough done." << endl;

	    // CREATING THE KDTREE OBJECT OFR THE SEARCH METHOD OF THE EXTRACTION
	    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); //for clustering
	    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
	    tree->setInputCloud (registration.cloud);

	    std::vector<pcl::PointIndices> cluster_indices;
	    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	    ec.setClusterTolerance (0.4); // 
	    ec.setMinClusterSize (30);
	    ec.setMaxClusterSize (200);
	    ec.setSearchMethod (tree);
	    ec.setInputCloud (registration.cloud);
	    ec.extract (cluster_indices);

	    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> points_cluster;
	    for (int ii=0;ii<cluster_indices.size ();ii++)
	    {
	        points_cluster.push_back(cloud_middle);
	    }

	    int j = 0;
	    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)   
	    {
	        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	        cloud_cluster->points.push_back (registration.cloud->points[*pit]); //*
	        cloud_cluster->width = cloud_cluster->points.size ();
	        cloud_cluster->height = 1;
	        cloud_cluster->is_dense = true;
	        points_cluster[j] = cloud_cluster;
	        j++;
	    }

	    for (int i=0; i<points_cluster.size(); i++)
	    {
	        ne.setSearchMethod (tree2);
	        ne.setInputCloud (points_cluster[i]);
	        ne.setKSearch (10);
	        ne.compute (*cloud_normals);

	        cout << "cluster " << i << " has " << cloud_normals->points.size() << " points." << endl;

	        // CREATE THE SEGMENTATION OBJECT FOR CYLINDER SEGMENTATION AND SET ALL THE PARAMETERS
	        seg.setOptimizeCoefficients (true);
	        seg.setModelType (pcl::SACMODEL_CYLINDER);
	        seg.setMethodType (pcl::SAC_RANSAC);
	        seg.setNormalDistanceWeight (0.01);
	        seg.setMaxIterations (500);
	        seg.setDistanceThreshold (0.05);
	        seg.setRadiusLimits (0.135,0.17);
	        seg.setInputCloud (points_cluster[i]);
	        seg.setInputNormals (cloud_normals);

	        // OBTAIN THE CYLINDER INLIERS AND COEFFICIENTS
	        seg.segment (*inliers_cylinder, *coefficients_cylinder);
	        extract.setInputCloud (points_cluster[i]);
	        extract.setIndices (inliers_cylinder);
	        extract.setNegative (false);
	        extract.filter (*cloud_cylinder);
	        //cout << cloud_cylinder->points.size() << " points can be fitted into a cylinder model from cluter " << i << "."<<  endl;

	        // IF 80% POINTS IN A CLUSTER CAN BE FITTED INTO A CYLINDER MODEL, THEN HOME CYLINDER IS DETECTED
	        if(float(cloud_cylinder->points.size())/float(cloud_normals->points.size()) >= 0.8)
	        {
	            cout << "cluster " << i << " has fit the cylinder model!!!" << endl;
	            cout << "Model coefficients: " << coefficients_cylinder->values[0] << " "
	                                           << coefficients_cylinder->values[1] << " "
	                                           << coefficients_cylinder->values[2] << " "
	                                           << coefficients_cylinder->values[3] << " "
	                                           << coefficients_cylinder->values[4] << " "
	                                           << coefficients_cylinder->values[5] << " "
	                                           << coefficients_cylinder->values[6] << endl;
	            cout << "Probability is " << seg.getProbability () << endl;

	            // PUBLISH THE CLUSTER LOCATION INFO

	        }   
	    }
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}