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

int map_range = 40; //size of local map is 20x20 m
float grid_size = 1; // size of the local map grid
float threshold_tree_height = 10.0; // above which the points will be disgarded
float home_detection_range = 15.0;
std::string fileName;

struct cylinder
{
	arma::mat points;
	arma::mat point_in_space = arma::zeros<arma::mat>(3,1);
	arma::mat axis_direction = arma::zeros<arma::mat>(3,1);
	arma::mat raius_estimate = arma::zeros<arma::mat>(1,1);
};


class NavigationFilter
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	void navigationFilterCallback(const messages::NavFilterOut::ConstPtr &msg)
	{
        this->x = msg->x_position; //meters
        this->y = msg->y_position; //meters
        this->roll = msg->roll; //radians
        this->pitch = msg->pitch; //radians
        this->heading = msg->heading; //radians
	}
public:
	float x;
	float y;
	float roll;
	float pitch;
	float heading;
	NavigationFilter()
	{
        x=0;
        y=0;
        roll=0;
        pitch=0;
        heading=0;    
		sub = nh.subscribe("navigation/navigationfilterout/navigationfilterout", 1, &NavigationFilter::navigationFilterCallback, this);
	}
};

class Registration
{
private:
	ros::NodeHandle nn;
	ros::Subscriber sub_laser;
	void registrationCallback(pcl::PointCloud<pcl::PointXYZ> const &input_cloud)
	{

	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            *cloud = input_cloud;
        ROS_INFO("\n*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*");
	    ROS_INFO("Running callback function with %i points.",cloud->points.size());
		
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

		// std::stringstream ss1;
		// ss1 << "raw_cloud.pcd";
		// pcl::io::savePCDFile( ss1.str(), *cloud);

		
	    // PASS THROUGH FILTER
	    pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud);
	    pass.setFilterFieldName("x");
	    pass.setFilterLimits(-map_range,map_range);
	    pass.filter(*cloud);
	    pass.setFilterFieldName("y");
	    pass.setFilterLimits(-map_range,map_range);
	    pass.filter(*cloud);
	    pass.setFilterFieldName("z");
	    pass.setFilterLimits(-1.5,threshold_tree_height);
	    pass.filter(*cloud);
	    //ROS_INFO("Regional cloud has %i points", cloud->points.size());


		// std::stringstream ss2;
		// ss2 << "middle_1.pcd";
		// pcl::io::savePCDFile( ss2.str(), *cloud);

	    /*
	    // CREATE THE FILTERING OBJECT
	    pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
	    pmf.setInputCloud (cloud);
	    pmf.setMaxWindowSize (20);
	    pmf.setSlope (1.0f);
	    pmf.setInitialDistance (0.5f);
	    pmf.setMaxDistance (1.5f);
	    //cout << "Cell size is " << pmf.getCellSize() << endl;
	    pmf.extract (ground->indices);

	    // CREATE THE FILTERING OBJECT
	    pcl::ExtractIndices<pcl::PointXYZ> extract;
	    extract.setInputCloud (cloud);
	    extract.setIndices (ground);
	    extract.filter (*ground_filtered);
	    */

	    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	    // Create the segmentation object
	    pcl::SACSegmentation<pcl::PointXYZ> seg_plane;
	    // Optional
	    seg_plane.setOptimizeCoefficients (true);
	    // Mandatory
	    seg_plane.setModelType (pcl::SACMODEL_PLANE);
	    seg_plane.setMethodType (pcl::SAC_RANSAC);
	    seg_plane.setMaxIterations (1000);
	    seg_plane.setDistanceThreshold (0.15); //Ground detection threshold parameter
	    seg_plane.setInputCloud (cloud); //was raw_cloud
	    seg_plane.segment (*inliers, *coefficients);
	    pcl::ExtractIndices<pcl::PointXYZ> extract;
	    extract.setInputCloud (cloud);
	    extract.setIndices (inliers);
	    extract.setNegative (false);
	    extract.filter (*ground_filtered);
	    extract.setNegative (true);
	    extract.filter (*object_filtered);
	    
	    

		// std::stringstream ss3;
		// ss3 << "ground.pcd";
		// pcl::io::savePCDFile( ss3.str(), *ground_filtered);

	    // EXTRACT NON-GROUND RETURNS
	    *object_filtered_projection = *object_filtered;
	    // PROJECTION, MAY NEED TO MODIFY THE ALGORITHM LATER
	    for (int i=0; i<object_filtered->points.size(); i++)
	    {
	      object_filtered_projection->points[i].z=0;
	    }
	    
	    // BUILD LOCAL MAP, THE CURRENT ALGORITHM IS VERY UN-EFFECTIVE, NEED TO IMPROVE LATER
		std::vector<float> point;
	    std::vector<std::vector<std::vector<float> > > grid_map_cells((map_range*2)*(map_range*2));
	    int index = 0;
	    local_grid_map.clear();
	    for (int i = 0; i< object_filtered->points.size(); i++)
	    {
	        point.push_back(object_filtered->points[i].x);
	        point.push_back(object_filtered->points[i].y);
	        point.push_back(object_filtered->points[i].z);
	        index = floor(object_filtered->points[i].x + map_range)*map_range + floor(object_filtered->points[i].y + map_range);
	        grid_map_cells[index].push_back(point);
	        point.clear();
	    }
	    float total_x = 0;
	    float total_y = 0;
	    float total_z = 0;
	    float average_x = 0;
	    float average_y = 0;
	    float average_z = 0;
	    float variance_z = 0;
	    for (int i = 0; i < grid_map_cells.size(); i++) // for every cell
	    {
	        for (int j = 0; j < grid_map_cells[i].size(); j++)
	        {
	            total_x += grid_map_cells[i][j][0];
	            total_y += grid_map_cells[i][j][1];
	            total_z += grid_map_cells[i][j][2];
	        }
	        average_x = total_x/grid_map_cells[i].size();
	        average_y = total_y/grid_map_cells[i].size();
	        average_z = total_z/grid_map_cells[i].size();
	        for (int j = 0; j < grid_map_cells[i].size(); j++)
	        {
	            variance_z = (grid_map_cells[i][j][2]-average_z) * (grid_map_cells[i][j][2]-average_z);
	        }
	        variance_z = sqrt(variance_z);
	        if (total_x || total_y || total_z)
	        {
	            // switch the coordinate of the LIDAR
	            float temp_holder = 0.0;
	            average_z = -average_z;
	            temp_holder = average_x;
	            average_x = average_y;
	            average_y = temp_holder;
	            point.push_back(average_x);
	            point.push_back(average_y);
	            point.push_back(average_z);
	            point.push_back(variance_z);
	            local_grid_map.push_back(point);
	            point.clear();
	        }
	        //print out points
	        if (total_x || total_y || total_z)
	            //cout << average_x << " " << average_y << " " << average_z << " " << variance_z <<  endl;
	        total_x = 0;
	        total_y = 0;
	        total_z = 0;
	        average_x = 0;
	        average_y = 0;
	        average_z = 0;
	        variance_z = 0;
	    }

		// THE local_grid HAS ALL INFO, NEED TO PUBLISH IT
		//TRANSFER POINTCLOUD TO POINTCLOUD2
		// pcl::PCLPointCloud2 tmp_cloud;
		// pcl::toPCLPointCloud2(*object_filtered_projection,tmp_cloud);
		// pcl_conversions::fromPCL(tmp_cloud, local_map_SLAM);

		// FROM HERE, IS THE HOME BEACON CYLINDER DETECTION PART
		// ONLY KEEP POINTS WITHIN THE HOME DETECTION RANGE
		pass.setInputCloud(object_filtered);
	    pass.setFilterFieldName("z");
	    pass.setFilterLimits(-1.5,3); 
	    pass.filter(*object_filtered);
	    pass.setInputCloud(object_filtered);
	    pass.setFilterFieldName("x");
	    pass.setFilterLimits(-home_detection_range,home_detection_range);
	    pass.filter(*object_filtered);
	    pass.setInputCloud(object_filtered);
	    pass.setFilterFieldName("y");
	    pass.setFilterLimits(-home_detection_range,home_detection_range); 
	    pass.filter(*object_filtered);
	    //ROS_INFO("PassThrough done.");
		
		// std::stringstream ss;
		// ss << "file.pcd";
		// pcl::io::savePCDFile( ss.str(), *object_filtered);

	    // CREATING THE KDTREE OBJECT OFR THE SEARCH METHOD OF THE EXTRACTION
	    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); //for clustering
	    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
	    tree->setInputCloud (object_filtered);

	    //cout << "0" << endl;

	    std::vector<pcl::PointIndices> cluster_indices;
	    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	    ec.setClusterTolerance (0.3); // 
	    ec.setMinClusterSize (50);
	    ec.setMaxClusterSize (5000);
	    ec.setSearchMethod (tree);
	    ec.setInputCloud (object_filtered);
	    ec.extract (cluster_indices);

	    //cout << "1" << endl;

	    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> points_cluster;
	    for (int ii=0;ii<cluster_indices.size ();ii++)
	    {
	        points_cluster.push_back(cloud_middle);
	    }
	    
		    //cout << "2" << endl;

	    int j = 0;
	    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)   
	    {
	        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	        cloud_cluster->points.push_back (object_filtered->points[*pit]); //*
	        cloud_cluster->width = cloud_cluster->points.size ();
	        cloud_cluster->height = 1;
	        cloud_cluster->is_dense = true;
	        points_cluster[j] = cloud_cluster;
	        j++;
	    }

	    std::vector<cylinder> cylinders;
	    cylinders.clear();
	    //loop through clusters
	    //ROS_INFO("%i clusters extracted from scan.", points_cluster.size());
        static bool stopSavingDataToFile = false;
        bool cylinderWasDetected = false;
		std::ofstream outputFile;
		if(stopSavingDataToFile==false)
		{
			outputFile.open(fileName.c_str(), ofstream::out | ofstream::trunc);
		}
	    for (int i=0; i<points_cluster.size(); i++)
	    {
	        ne.setSearchMethod (tree2);
	        ne.setInputCloud (points_cluster[i]);
	        ne.setKSearch (25);
	        ne.compute (*cloud_normals);

	        //ROS_INFO("cluster %i has %i points", i, cloud_normals->points.size());

	        // CREATE THE SEGMENTATION OBJECT FOR CYLINDER SEGMENTATION AND SET ALL THE PARAMETERS
	        seg.setOptimizeCoefficients (true);
	        seg.setModelType (pcl::SACMODEL_CYLINDER);
	        seg.setMethodType (pcl::SAC_RANSAC);
	        seg.setNormalDistanceWeight (0.01);
	        seg.setMaxIterations (1000);
	        seg.setDistanceThreshold (0.05);
	        seg.setRadiusLimits (0.14,0.17);
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
	        cylinder current_cylinder;
	        if(float(cloud_cylinder->points.size())/float(cloud_normals->points.size()) >= 0.9)
	        {
	        	cylinderWasDetected = true;
	            ROS_INFO("cluster %i has the size of %i and has fit the cylinder model with probability %f", i, cloud_normals->points.size(), seg.getProbability());
	             cout << "Model coefficients: " << coefficients_cylinder->values[0] << " "
	                                            << coefficients_cylinder->values[1] << " "
	                                            << coefficients_cylinder->values[2] << " "
	                                            << coefficients_cylinder->values[3] << " "
	                                            << coefficients_cylinder->values[4] << " "
	                                            << coefficients_cylinder->values[5] << " "
	                                            << coefficients_cylinder->values[6] << endl;
	            cout << "Probability is " << seg.getProbability () << endl;


			float max_x = cloud_cylinder->points[0].x;
		    float max_y = cloud_cylinder->points[0].y;
		    float min_x = cloud_cylinder->points[0].x;
		    float min_y = cloud_cylinder->points[0].y;
		    for (int jj=1; jj<cloud_cylinder->points.size();jj++)
		    {
			if(cloud_cylinder->points[jj].x > max_x)
			    max_x = cloud_cylinder->points[jj].x;
			if(cloud_cylinder->points[jj].y > max_y)
			    max_y = cloud_cylinder->points[jj].y;
			if(cloud_cylinder->points[jj].x < min_x)
			    min_x = cloud_cylinder->points[jj].x;
			if(cloud_cylinder->points[jj].y < min_y)
			    min_y = cloud_cylinder->points[jj].y;
		    }
				if(abs(max_x-min_x)<0.6 && abs(max_y-min_y)<0.6)
				{
		    	current_cylinder.point_in_space(0,0) =  (double)coefficients_cylinder->values[0];
		    	current_cylinder.point_in_space(1,0) = -(double)coefficients_cylinder->values[1];
		    	current_cylinder.point_in_space(2,0) = -(double)coefficients_cylinder->values[2];
		    	current_cylinder.axis_direction(0,0) =  (double)coefficients_cylinder->values[3];
		    	current_cylinder.axis_direction(1,0) = -(double)coefficients_cylinder->values[4];
		    	current_cylinder.axis_direction(2,0) = -(double)coefficients_cylinder->values[5];
		    	current_cylinder.raius_estimate(0,0) =  (double)coefficients_cylinder->values[6];
			    	
			    current_cylinder.points.zeros(3,cloud_cylinder->points.size());
			    for (int jj=0; jj<cloud_cylinder->points.size(); jj++)
			    {
			    	current_cylinder.points(0,jj)= (double)cloud_cylinder->points[jj].x;
			    	current_cylinder.points(1,jj)=-(double)cloud_cylinder->points[jj].y;
			    	current_cylinder.points(2,jj)=-(double)cloud_cylinder->points[jj].z;

			    	if(stopSavingDataToFile==false)
			    	{
				    	outputFile << current_cylinder.points(0,jj) << ",";
				    	outputFile << current_cylinder.points(1,jj) << ",";
				    	outputFile << current_cylinder.points(2,jj) << ",";
				    	outputFile << i << ","; //cylinder number
				    	outputFile << current_cylinder.point_in_space(0,0) << ",";
				    	outputFile << current_cylinder.point_in_space(1,0) << ",";
				    	outputFile << current_cylinder.point_in_space(2,0) << ",";
				    	outputFile << current_cylinder.axis_direction(0,0) << ",";
				    	outputFile << current_cylinder.axis_direction(1,0) << ",";
				    	outputFile << current_cylinder.axis_direction(2,0) << ",";
				    	outputFile << current_cylinder.raius_estimate(0,0);
				    	outputFile << std::endl;   		
			    	}
			    }
			    //cout << "current_cylinder.points size = " << current_cylinder.points.n_rows << ", " << current_cylinder.points.n_cols << endl;
			    cylinders.push_back(current_cylinder);
				}
	        }   
	    }
 
	    //begin homing
	    if(cylinders.size()>=2) 
	    {
		    // find correct cylinders
		    arma::mat xs1;
		    arma::mat xs2;
		    arma::mat ys1;
		    arma::mat ys2;
		    arma::mat R(3,3); // Roll Matrix
			arma::mat P(3,3); // Pitch Matrix
			arma::mat Rot(3,3); // Total Rotation Matrix
			double dist = 2.0-12.0*0.0254;
			double r = 6.0*0.0254;
			double t, c1_x, c1_y, c2_x, c2_y, x, y, ax1, ay1, ax2, ay2, x_mean, y_mean, d, bearing;
			double v1_x, v1_y, v2_x, v2_y, v1_mag, v2_mag, v_dot, X1s_x, X1s_y, X2s_x, X2s_y, X1s_mag, X2s_mag;
			double cx1, cx2, cy1, cy2;
			bool cylinder_found = false;

			// rotate points
			// Define roll matrix
			double phi = 0.0;
			double theta = 0.0;
			R(1,1) = cos(phi);
			R(1,2) = sin(phi);	
			R(2,1) = -sin(phi);
			R(2,2) = cos(phi);
			R(0,0) = 1.0;
			R(0,1) = 0.0;
			R(1,0) = 0.0;
			R(0,2) = 0.0;
			R(2,0) = 0.0;
			
			// Define pitch matrix
			P(0,0) = cos(theta);
			P(0,2) = -sin(theta);	
			P(2,0) = sin(theta);
			P(2,2) = cos(theta);
			P(1,0) = 0.0;
			P(0,1) = 0.0;
			P(1,1) = 1.0;
			P(1,2) = 0.0;
			P(2,1) = 0.0;
		    
		    // find total rotation
			Rot = R*P;

			// rotate points and transform cylinder parameters
		    for (int ii=0; ii<cylinders.size(); ii++)
		    {
		    	// rotate points
		    	//cylinders[ii].points.print("cylinders[ii].points = ");
		    	//cout << "points_cluster.size() = " << points_cluster.size() << endl;
		    	//cout << "cylinders[" << ii << "].points size = " << cylinders[ii].points.n_rows << ", " << cylinders[ii].points.n_cols << endl;
		    	cylinders[ii].points = Rot*cylinders[ii].points;

		    	// rotate cylinder parameters
		    	cylinders[ii].point_in_space = Rot*cylinders[ii].point_in_space;
		    	cylinders[ii].axis_direction = Rot*cylinders[ii].axis_direction;

		    	// translate point to x_y plane along axis direction
			if (cylinders[ii].axis_direction(2,0)!=0)
			{
			    	t = -cylinders[ii].point_in_space(2,0)/cylinders[ii].axis_direction(2,0);
			    	cylinders[ii].point_in_space(0,0) = cylinders[ii].point_in_space(0,0)+t*cylinders[ii].axis_direction(0,0);
			    	cylinders[ii].point_in_space(1,0) = cylinders[ii].point_in_space(1,0)+t*cylinders[ii].axis_direction(1,0);
			    	cylinders[ii].point_in_space(2,0) = 0.0;
			}
		    }

		    //cout << "1" << endl;
		    // find all cylinders correct distance apart
			for (int ii=0; ii<cylinders.size()-1; ii++)
		    {
		    	//cout << "2" << endl;
		    	c1_x = cylinders[ii].point_in_space(0,0);
		    	c1_y = cylinders[ii].point_in_space(1,0);
		    	for (int jj=ii+1; jj<cylinders.size(); jj++)
		    	{
		    		//cout << "3" << endl;
		    		c2_x = cylinders[jj].point_in_space(0,0);
		    		c2_y = cylinders[jj].point_in_space(1,0);
		    		if (abs(sqrt((c1_x-c2_x)*(c1_x-c2_x)+(c1_y-c2_y)*(c1_y-c2_y))-dist)<0.05)
		    		{
		    			cylinder_found = true;
		    			if (c1_x*c2_y-c2_x*c1_y>0)
		    			{
		    				xs1 = cylinders[ii].points.row(0);
		    				ys1 = cylinders[ii].points.row(1);
		    				xs2 = cylinders[jj].points.row(0);
		    				ys2 = cylinders[jj].points.row(1);
						cx1 = c1_x;
						cy1 = c1_y;
						cx2 = c2_x;
						cy2 = c2_y;
		    			}
		    			else
		    			{
		    				xs1 = cylinders[jj].points.row(0);
		    				ys1 = cylinders[jj].points.row(1);
		    				xs2 = cylinders[ii].points.row(0);
		    				ys2 = cylinders[ii].points.row(1);
						cx1 = c2_x;
						cy1 = c2_y;
						cx2 = c1_x;
						cy2 = c1_y;
		    			}
		    		}
		    	}
		    }

		    //cout << "4" << endl;
		    // do the fit
			int n1 = xs1.n_cols;
			int n2 = xs2.n_cols;
			arma::mat X(4,1);
			arma::mat FX(n1+n2+1,1);
			arma::mat J(n1+n2+1,4);
			arma::mat W(n1+n2+1,n1+n2+1,arma::fill::eye);
			W(n1+n2,n1+n2) = 10000;

			if (cylinder_found)
			{
				//X(0,0) = cx1; //column 1 x-center
				//X(1,0) = cy1; //column 1 y-center
				//X(2,0) = cx2; //column 2 x-center
				//X(3,0) = cy2; //column 2 y-center
				//ROS_INFO("Cylinder centroids = %f, %f, %f, %f", c1_x, c1_y, c2_x, c2_y);
				// alternate initial guess
					double X1s_x = arma::as_scalar(arma::mean(xs1,1));
					double X1s_y = arma::as_scalar(arma::mean(ys1,1));
					double X2s_x = arma::as_scalar(arma::mean(xs2,1));
					double X2s_y = arma::as_scalar(arma::mean(ys2,1));
					// cout<<"X1s_x="<<X1s_x<<endl;
					// cout<<"X1s_y="<<X1s_y<<endl;
					// cout<<"X2s_x="<<X2s_x<<endl;
					// cout<<"X2s_y="<<X2s_y<<endl;
					double X1s_mag = sqrt(X1s_x*X1s_x+X1s_y*X1s_y);
					double X2s_mag = sqrt(X2s_x*X2s_x+X2s_y*X2s_y);
					X1s_x = X1s_x+r*X1s_x/X1s_mag;
					X1s_y = X1s_y+r*X1s_y/X1s_mag;
					X2s_x = X2s_x+r*X2s_x/X2s_mag;
					X2s_y = X2s_y+r*X2s_y/X2s_mag;
					X(0) = X1s_x; //column 1 x-center
					X(1) = X1s_y; //column 1 y-center
					X(2) = X2s_x; //column 2 x-center
					X(3) = X2s_y; //column 2 y-center

				for (int ii = 0; ii<100; ii++)
				{
					ax1 = X(0,0);
					ay1 = X(1,0);
					ax2 = X(2,0);
					ay2 = X(3,0);

					for (int jj = 0; jj<n1; jj++)
					{
						x = xs1(0,jj);
						y = ys1(0,jj);
						FX(jj,0) = (x-ax1)*(x-ax1)+(y-ay1)*(y-ay1)-r*r;
						J(jj,0) = 2*ax1-2*x;
						J(jj,1) = 2*ay1-2*y;
					}
					for (int jj = n1; jj<n1+n2; jj++)
					{
						x = xs2(0,jj-n1);
						y = ys2(0,jj-n1);
						FX(jj,0) = (x-ax2)*(x-ax2)+(y-ay2)*(y-ay2)-r*r;
						J(jj,2) = 2*ax2-2*x;
						J(jj,3) = 2*ay2-2*y;
					}
					FX(n1+n2,0) = (ax1-ax2)*(ax1-ax2)+(ay1-ay2)*(ay1-ay2)-dist*dist;
					J(n1+n2,0) = 2*ax1-2*ax2;
					J(n1+n2,1) = 2*ay1-2*ay2;
					J(n1+n2,2) = 2*ax2-2*ax1;
					J(n1+n2,3) = 2*ay2-2*ay1;
					X = X-0.25*solve(J.st()*W*J,J.st()*W*FX);
				}

				//cout << "5" << endl;
				x_mean = (X(0,0)+X(2,0))/2;
				y_mean = (X(1,0)+X(3,0))/2;

				//x_mean = (cx1+cx2)/2;
				//y_mean = (cy1+cy2)/2;
				d = sqrt(x_mean*x_mean+y_mean*y_mean);

				v2_x = X(0,0)-X(2,0);
				v2_y = X(1,0)-X(3,0);
				//v2_x = cx1-cx2;
				//v2_y = cy1-cy2;
				v1_mag = sqrt(x_mean*x_mean+y_mean*y_mean); 
				v2_mag = sqrt(v2_x*v2_x+v2_y*v2_y); 
				v1_x = x_mean/v1_mag;
				v1_y = y_mean/v1_mag;
				v2_x = v2_x/v2_mag;
				v2_y = v2_y/v2_mag;

				v_dot = v1_x*v2_x+v1_y*v2_y;
				bearing = acos(v_dot)-3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798214808651/2;
				double x_est = d*cos(bearing);
				double y_est = d*sin(bearing);
				double b_h_diff = atan2(-v1_y,-v1_x); 
				double heading_est = -(b_h_diff-bearing);

				ROS_INFO("\nHOMING UPDATE!");
				ROS_INFO("x = %f", x_est);
				ROS_INFO("y = %f", y_est);
				ROS_INFO("heading = %f", heading_est*180.0/3.14159265);

				//homing_x=x_est;
				//homing_y=y_est;
				homing_x = x_mean;
				homing_y = y_mean;
				homing_heading=heading_est;
				homing_found=true;
				ROS_INFO("********************");
				ROS_INFO("x_est = %f",x_est);
				ROS_INFO("y_est = %f",y_est);
				ROS_INFO("x_mean = %f",x_mean);
				ROS_INFO("y_mean = %f",y_mean);
				ROS_INFO("heading = %f\n",heading_est);
			}
			else
			{
				homing_x=0;
				homing_y=0;
				homing_heading=0;
				homing_found=false;
			}
			// end homing

			if(stopSavingDataToFile==false && homing_found==true)
			{
				outputFile.close();
				stopSavingDataToFile=true; 
			}
			else
			{
				outputFile.close();
			}
		}
		counter++;
	}
public:
	float homing_x=0;
	float homing_y=0;
	float homing_heading=0;
	bool homing_found=false;
	int counter=0;

    //x, y, heading, roll, and pitch from the navigation filter node
	float robot_x=0;
	float robot_y=0;
	float robot_roll=0;
	float robot_pitch=0;
	float robot_heading=0;
	
	//local map output for slam node
	std::vector<float> x_mean;
	std::vector<float> y_mean;
	std::vector<float> z_mean;
	std::vector<float> var_z;
	std::vector<bool> ground_adjacent;
	std::vector<std::vector<float> > local_grid_map;
	// float copied_robot_x;
	// float copied_robot_y;
	// float copied_robot_heading;
	
	
	// sensor_msgs::PointCloud2 local_map_SLAM;
	Registration()
	{
		sub_laser = nn.subscribe("/velodyne_points", 1, &Registration::registrationCallback, this);

	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_filtering_node");
	ROS_INFO("lidar_filtering_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);
	ros::Publisher pub_lidar = nh.advertise<messages::LidarFilterOut>("lidar/lidarfilteringout/lidarfilteringout",1);
	//publish 
	//ros::Publisher pub_local_map_SLAM = nh.advertise<sensor_msgs::PointCloud2>("cloud_pcd",1);
	ros::Publisher pub_local_map = nh.advertise<messages::LocalMap>("/lidar/lidarfilteringnode/localmap",1);


	fileName = "points_from_cylinders_" + patch::currentDateTime() + ".txt";

	//subscriber and algorithm
	Registration registration; 
	NavigationFilter navigation_filter;

	//output messages
	messages::LidarFilterOut msg_LidarFilterOut;
	messages::LocalMap msg_LocalMap;

	//new message counter
	int prev_counter = registration.counter;
	int publish_counter = 0;
	while(ros::ok())
	{
	    //update registration variables to navigation filter pose
	    registration.robot_x = navigation_filter.x; //meters
	    registration.robot_y = navigation_filter.y; //meters
	    registration.robot_roll = navigation_filter.roll; //radians
	    registration.robot_pitch = navigation_filter.pitch; //radians
	    registration.robot_heading = navigation_filter.heading; //radians
	    
	
		if(prev_counter != registration.counter)
		{
			//populate message for homing
			msg_LidarFilterOut.homing_x = registration.homing_x;
			msg_LidarFilterOut.homing_y = registration.homing_y;
			msg_LidarFilterOut.homing_heading = registration.homing_heading;
			msg_LidarFilterOut.homing_found = registration.homing_found;
			
			//populate message for local map
			msg_LocalMap.x_mean.clear();
			msg_LocalMap.y_mean.clear();
			msg_LocalMap.z_mean.clear();
			msg_LocalMap.var_z.clear();
			msg_LocalMap.ground_adjacent.clear();
			for(int i=0; i<registration.local_grid_map.size(); i++)
			{
			    msg_LocalMap.x_mean.push_back(registration.local_grid_map[i][0]);
			    msg_LocalMap.y_mean.push_back(registration.local_grid_map[i][1]);
			    msg_LocalMap.z_mean.push_back(registration.local_grid_map[i][2]);
			    msg_LocalMap.var_z.push_back(registration.local_grid_map[i][3]);
			    msg_LocalMap.ground_adjacent.push_back(1);
			}
			msg_LocalMap.x_filter = registration.robot_x;
			msg_LocalMap.y_filter = registration.robot_y;
			msg_LocalMap.heading_filter = registration.robot_heading;
			msg_LocalMap.new_data = true;
		}
		else
		{
			msg_LidarFilterOut.homing_x = 0;
			msg_LidarFilterOut.homing_y = 0;
			msg_LidarFilterOut.homing_heading = 0;
			msg_LidarFilterOut.homing_found = 0;
			
			//if no new data fill message with 0
			msg_LocalMap.x_mean.clear();
			msg_LocalMap.y_mean.clear();
			msg_LocalMap.z_mean.clear();
			msg_LocalMap.var_z.clear();
			msg_LocalMap.ground_adjacent.clear();
			msg_LocalMap.x_filter = 0;
			msg_LocalMap.y_filter = 0;
			msg_LocalMap.heading_filter = 0;	
			msg_LocalMap.new_data = false;	
		}

		prev_counter = registration.counter;

		//publish message
		pub_lidar.publish(msg_LidarFilterOut);
		pub_local_map.publish(msg_LocalMap);
		/*
		if(publish_counter >= 10)
		{
			pub_local_map_SLAM.publish(registration.local_map_SLAM);
			publish_counter = 0;
		}
		publish_counter++;
		*/
		loop_rate.sleep();
		ros::spinOnce();		
	}

	return 0;
}
