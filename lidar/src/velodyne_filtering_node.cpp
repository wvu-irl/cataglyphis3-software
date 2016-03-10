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

using namespace std;

int map_range = 40; //size of local map is 20x20 m
float grid_size = 1; // size of the local map grid
float threshold_tree_height = 10.0; // above which the points will be disgarded
float home_detection_range = 15.0;

struct cylinder
{
	arma::mat points;
	arma::mat point_in_space = arma::zeros<arma::mat>(3,1);
	arma::mat axis_direction = arma::zeros<arma::mat>(3,1);
	arma::mat raius_estimate = arma::zeros<arma::mat>(1,1);
};

class Registration
{
private:
	ros::NodeHandle nn;
	ros::Subscriber sub_laser;
	int counter = 0;

	void registrationCallback(pcl::PointCloud<pcl::PointXYZ> const &input_cloud)
	{
		if(counter > 2)
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
			// vector < pcl::PointCloud<pcl::PointXYZ>::Ptr > local_grid;
			// cout << "Object after projection has " << object_filtered_projection->points.size() << " points." << endl;
			// for (int i=0; i<(2*map_range)*(2*map_range); i++) // generate gird element
			// {
			// 	pass.setInputCloud(object_filtered_projection);
			// 	pass.setFilterFieldName("y");
			// 	pass.setFilterLimits(-map_range + floor(i/(2*map_range)) , -map_range + floor((i/2*map_range)) + grid_size); //ML direction
			// 	pass.filter(*grid_cloud);
			// 	pass.setInputCloud(grid_cloud);
			// 	pass.setFilterFieldName("x");
			// 	pass.setFilterLimits(-map_range + i % (2*map_range) , -map_range + i % (2*map_range) + grid_size); //ML direction
			// 	pass.filter(*grid_cloud);
			// 	//cout << "grid has " << grid_cloud->points.size() << " points" << endl;
			// 	//local_grid.push_back((*grid_cloud).make_shared());
			// 	local_grid.push_back(grid_cloud);
			// 	//cout << "Size of grid map " << local_grid.size() << endl;
			// 	cout << "Grid " << i << " has " << (local_grid[i])->points.size() << " points."<< endl;
			// }

			// THE local_grid HAS ALL INFO, NEED TO PUBLISH IT


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

		    vector<cylinder> cylinders;
		    cylinders.clear();
		    //loop through clusters
		    //ROS_INFO("%i clusters extracted from scan.", points_cluster.size());
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
		            ROS_INFO("cluster %i has fit the cylinder model with probability %f", i, seg.getProbability());
		            // cout << "Model coefficients: " << coefficients_cylinder->values[0] << " "
		            //                                << coefficients_cylinder->values[1] << " "
		            //                                << coefficients_cylinder->values[2] << " "
		            //                                << coefficients_cylinder->values[3] << " "
		            //                                << coefficients_cylinder->values[4] << " "
		            //                                << coefficients_cylinder->values[5] << " "
		            //                                << coefficients_cylinder->values[6] << endl;
		            //cout << "Probability is " << seg.getProbability () << endl;

			    	current_cylinder.point_in_space(0,0) = coefficients_cylinder->values[0];
			    	current_cylinder.point_in_space(1,0) = coefficients_cylinder->values[1];
			    	current_cylinder.point_in_space(2,0) = coefficients_cylinder->values[2];
			    	current_cylinder.axis_direction(0,0) = coefficients_cylinder->values[3];
			    	current_cylinder.axis_direction(1,0) = coefficients_cylinder->values[4];
			    	current_cylinder.axis_direction(2,0) = coefficients_cylinder->values[5];
			    	current_cylinder.raius_estimate(0,0) = coefficients_cylinder->values[6];
				    	
				    current_cylinder.points.zeros(3,cloud_cylinder->points.size());
				    for (int jj=0; jj<cloud_cylinder->points.size(); jj++)
				    {
				    	current_cylinder.points(0,jj)=object_filtered_projection->points[jj].x;
				    	current_cylinder.points(1,jj)=-object_filtered_projection->points[jj].y;
				    	current_cylinder.points(2,jj)=-object_filtered_projection->points[jj].z;
				    }
				    //cout << "current_cylinder.points size = " << current_cylinder.points.n_rows << ", " << current_cylinder.points.n_cols << endl;
				    cylinders.push_back(current_cylinder);
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
			    	t = -cylinders[ii].point_in_space(2,0)/cylinders[ii].axis_direction(2,0);
			    	cylinders[ii].point_in_space(0,0) = cylinders[ii].point_in_space(0,0)+t*cylinders[ii].axis_direction(0,0);
			    	cylinders[ii].point_in_space(1,0) = cylinders[ii].point_in_space(1,0)+t*cylinders[ii].axis_direction(1,0);
			    	cylinders[ii].point_in_space(2,0) = 0.0;
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
			    		if (abs(sqrt((c1_x-c2_x)*(c1_x-c2_x)+(c1_y-c2_y)*(c1_y-c2_y))-dist)<0.2)
			    		{
			    			cylinder_found = true;
			    			if (c1_y<c2_y)
			    			{
			    				xs1 = cylinders[ii].points.row(0);
			    				ys1 = cylinders[ii].points.row(1);
			    				xs2 = cylinders[jj].points.row(0);
			    				ys2 = cylinders[jj].points.row(1);
			    			}
			    			else
			    			{
			    				xs1 = cylinders[jj].points.row(0);
			    				ys1 = cylinders[jj].points.row(1);
			    				xs2 = cylinders[ii].points.row(0);
			    				ys2 = cylinders[ii].points.row(1);
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
					X(0,0) = c1_x; //column 1 x-center
					X(1,0) = c1_y; //column 1 y-center
					X(2,0) = c2_x; //column 2 x-center
					X(3,0) = c2_y; //column 2 y-center
					//ROS_INFO("Cylinder centroids = %f, %f, %f, %f", c1_x, c1_y, c2_x, c2_y);
					// alternate initial guess
						// double X1s_x = arma::as_scalar(arma::mean(xs1,1));
						// double X1s_y = arma::as_scalar(arma::mean(ys1,1));
						// double X2s_x = arma::as_scalar(arma::mean(xs2,1));
						// double X2s_y = arma::as_scalar(arma::mean(ys2,1));
						// cout<<"X1s_x="<<X1s_x<<endl;
						// cout<<"X1s_y="<<X1s_y<<endl;
						// cout<<"X2s_x="<<X2s_x<<endl;
						// cout<<"X2s_y="<<X2s_y<<endl;
						// double X1s_mag = sqrt(X1s_x*X1s_x+X1s_y*X1s_y);
						// double X2s_mag = sqrt(X2s_x*X2s_x+X2s_y*X2s_y);
						// X1s_x = X1s_x+r*X1s_x/X1s_mag;
						// X1s_y = X1s_y+r*X1s_y/X1s_mag;
						// X2s_x = X2s_x+r*X2s_x/X2s_mag;
						// X2s_y = X2s_y+r*X2s_y/X2s_mag;
						// X(0) = X1s_x; //column 1 x-center
						// X(1) = X1s_y; //column 1 y-center
						// X(2) = X2s_x; //column 2 x-center
						// X(3) = X2s_y; //column 2 y-center

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
						// X = X-0.25*solve(J.st()*W*J,J.st()*W*FX);
					}

					//cout << "5" << endl;
					// x_mean = (X(0,0)+X(2,0))/2;
					// y_mean = (X(1,0)+X(3,0))/2;

					x_mean = (c1_x+c2_x)/2;
					y_mean = (c1_y+c2_y)/2;
					d = sqrt(x_mean*x_mean+y_mean*y_mean);

					// v2_x = X(0,0)-X(2,0);
					// v2_y = X(1,0)-X(3,0);
					v2_x = c1_x-c2_x;
					v2_y = c1_y+c2_y;
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

					ROS_INFO("HOMING UPDATE! x, y, heading = %f, %f, %f", x_est, y_est, heading_est);
				}
				else
				{

				}
				// end homing
			}
		    counter = 0;
		}
		counter++;
	}
public:
	Registration()
	{
		sub_laser = nn.subscribe("/velodyne_points", 1, &Registration::registrationCallback, this);

	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "velodyne_filtering_node");
	ros::NodeHandle nh;

	ROS_INFO("velodyne_filtering_node running...");

	Registration registration; 
	ros::spin();

	return 0;
}
