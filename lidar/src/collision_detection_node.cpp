#include <ros/ros.h>
#include <iostream>
#include <messages/CollisionOut.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

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
#include <armadillo>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//using namespace std;
//using namespace pcl;

const float PI = 3.14159265;
const float DEG2RAD = PI/180;
const float RAD2DEG = 180/PI;

float corridor_width = 1.5; // Width of the virtual corridor
float corridor_length = 10.0; // Length of the virtual corridor, THE CLOSEST DECTECTION IS ~2.8 M
float x_shift = 0.0;
float y_shift = 0.0;
float lidar_height = 0.76; // height of the lidar sensor
float safe_envelope_angle = DEG2RAD*15; //safe envelope angle size
int point_trigger_threshold = 5; // how many points that will trigger the avoid alarm

//subscribe to state machine info
class RobotControlSubscriber
{
private:
	// ros::Subscriber sub_sm;
	// ros::NodeHandle node;

	// void getExecCallback(const robot_control::ExecStateMachineInfo::ConstPtr &msg)
	// {
	// 	if(msg->var == 1) //logic to decide to run collision detection or not
	// 	{
	// 		this->check_for_collisions=1;
	// 	}
	// 	else
	// 	{
	// 		this->check_for_collisions=0;
	// 	}
	// }
public:
	int check_for_collisions = 1;
	// RobotControlSubscriber()
	// {
	// 	// sub_sm = node.subscribe("/control/execinfo/execinfo", 1, &RobotControlSubscriber::getExecCallback, this);
	// 	check_for_collisions = 1; //change to 0 once subscribed to robot control
	// }
};


class Registration
{
private:
	ros::NodeHandle nn;
	ros::Subscriber sub_laser;
	RobotControlSubscriber robot_control;
	int counter = 0;

	void registrationCallback(pcl::PointCloud<pcl::PointXYZI> const &input_cloud)
	{
		//check for collisions
		if(robot_control.check_for_collisions==1) 
		{
			//function trim laser down to only points in virtual corridor

			//do ground removal on points in virtual corridor
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>); // The raw point cloud from the LIDAR
			*cloud = input_cloud;
/*
			pcl::PointCloud<pcl::PointXYZ>::Ptr ground_filtered (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr object_filtered (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_middle (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointIndicesPtr ground (new pcl::PointIndices);

            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(y_shift - corridor_width,y_shift + corridor_width); 
            pass.filter(*cloud);
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(x_shift, x_shift + corridor_length);
            pass.filter(*cloud);
            //ROS_INFO("Virtual corridor has %i points.", cloud->points.size());

			
			// // //use rough ground removal, method A: rough 
			// // // A: start
			// // pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
			// // pmf.setInputCloud (cloud);
			// // pmf.setMaxWindowSize (20);
			// // pmf.setSlope (1.0f);
			// // pmf.setInitialDistance (0.3f);
			// // pmf.setMaxDistance (0.7f);
			// // pmf.extract (ground->indices);
			// // // CREATE THE FILTERING OBJECT
			// // pcl::ExtractIndices<pcl::PointXYZ> extract;
			// // extract.setInputCloud (cloud);
			// // // EXTRACT GROUND RETURNS
			// // extract.setIndices (ground);
			// // extract.filter (*ground_filtered);
			// // // EXTRACT NON-GROUND RETURNS
			// // extract.setNegative (true);
			// // extract.filter (*object_filtered);
			// // // A: end
			

			// // // B: start method B: find
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
			// Create the segmentation object
			pcl::SACSegmentation<pcl::PointXYZ> seg;
			// Optional
			seg.setOptimizeCoefficients (true);
			// Mandatory
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setMaxIterations (1000);
			seg.setDistanceThreshold (0.15); //Ground detection threshold parameter
			seg.setInputCloud (cloud);
			seg.segment (*inliers, *coefficients);
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (cloud);
			extract.setIndices (inliers);
			extract.setNegative (false);
			extract.filter (*ground_filtered);
			extract.setNegative (true);
			extract.filter (*object_filtered);
			//ROS_INFO("OBJECT FOUND BY LAYER 2, SIZE IS %i points.", object_filtered->points.size());

			
			// Statistical outlier removal
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(object_filtered);
            sor.setMeanK(5);
            sor.setStddevMulThresh(1.0);
            sor.filter(*object_filtered);
            //ROS_INFO("OBJECT FOUND BY LAYER 2 after statistical removal, SIZE IS %i points.", object_filtered->points.size());
            
            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); //for clustering
            tree->setInputCloud (object_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (0.3); // 
            ec.setMinClusterSize (3);   // 100-200 when the cylinder is 6 meters away
            ec.setMaxClusterSize (500);
            ec.setSearchMethod (tree);
            ec.setInputCloud (object_filtered);
            ec.extract (cluster_indices);

            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> points_cluster;
            for (int i=0;i<cluster_indices.size ();i++)
            {
              points_cluster.push_back(cloud_middle);
            }
			
			int j = 0;

            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)   
            {
              //points_cluster[j]= new pcl::PointCloud<pcl::PointXYZ>(); 
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
              for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
              cloud_cluster->points.push_back (object_filtered->points[*pit]); //*
              cloud_cluster->width = cloud_cluster->points.size ();
              cloud_cluster->height = 1;
              cloud_cluster->is_dense = true;

              //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
              points_cluster[j] = cloud_cluster;
              j++;
            }
            
            if(points_cluster.size() == 1)
            {
                //ROS_INFO("There is only 1 object in the region.");
                // Calculate the center and size, determine which way to go and aovid it
                Eigen::Vector4f centroid;
	            pcl::compute3DCentroid(*(points_cluster[0]), centroid);
	            if(centroid[1] < 0)
	            {
	                //ROS_INFO("LARGEST COLLISION ON RIGHT FROM LAYER 2");
	            }
	            else
	            {
	                //ROS_INFO("LARGEST COLLISION ON LEFT FROM LAYER 2");
	            }
            }
            else if(points_cluster.size() == 2)
            {
                //ROS_INFO("There are only 2 objects in the region.");
				// Eigen::Vector4f centroid1, centroid2;
				// pcl::compute3DCentroid(*(points_cluster[0]), centroid1);
				// pcl::compute3DCentroid(*(points_cluster[1]), centroid2);
				// if(centroid1 < 0 && centroid2 < 0) // both on right
				// {
				//     ROS_INFO("LARGEST COLLISION ON RIGHT FROM LAYER 2");
				// }
				// elseif(centroid1 > 0 && centroid2 > 0) // both on left
				// {
				//     ROS_INFO("LARGEST COLLISION ON LEFT FROM LAYER 2");
				// }
				// else // one on left, one on right
				// {
				//     //
				//     ROS_INFO("COLLISION ON BOTH SIDE FROM LAYER 2");
				// }
                
            }
            else
            {
                //ROS_INFO("There are %i objects in the region.", points_cluster.size());
                // 
            }
			// B: end
*/
			//check for collision
			int collision_point_counter = 0;
			int collision_left_counter = 0;
			int collision_right_counter = 0;
			// FIRST LAYER: SAFE ENVELOPE
			for (int i=0; i<cloud->points.size(); i++)
			{
				//cout << "x,y,z =" << cloud->points[i].x << ", " << cloud->points[i].y << ", " << cloud->points[i].z << endl;
				//CHECK IF POINT IN CORRIDOR

				if(cloud->points[i].x > 0 && cloud->points[i].y < 1 && cloud->points[i].y > -1 && cloud->points[i].x < 5)
				{
					//cout << cloud->points[i].x << " " << cloud->points[i].z << " " << fabs(atan2(cloud->points[i].z,cloud->points[i].x))*RAD2DEG << " " << safe_envelope_angle << endl;
					// CHECK IF THE POINT IS OUTSIDE OF THE SAFE ENVELOPE
					if(fabs(atan2( (lidar_height + cloud->points[i].z),cloud->points[i].x )) > safe_envelope_angle )
					//if(cloud->points[i].z > -0.2 && cloud->points[i].z  < 0.2)
					{
						//cout << "x,y,z =" << cloud->points[i].x << ", " << cloud->points[i].y << ", " << cloud->points[i].z << endl;
						// NEED TO AVOID
						collision_point_counter++;
						if(cloud->points[i].y<0)
						{
							collision_right_counter++;
						}
						else
						{
							collision_left_counter++;
						}
					} 
				}
			}

			if(collision_point_counter>10)
			{
				if(collision_left_counter > collision_right_counter)
				{
					collision=1;
					ROS_INFO("LARGEST COLLISION ON LEFT");
				}
				else
				{
					collision=2;
					ROS_INFO("LARGEST COLLISION ON RIGHT");				
				}
			}
			else
			{
				collision=0;
				ROS_INFO("No Collision...");
			}
			collision_point_counter = 0;
			collision_left_counter = 0;
			collision_right_counter = 0;


			//predictive 
			int preditive_region_length = 7;
			int preditive_region_wide = 5; //this is on each side, so the total is 5*2 =10

			//remove points based on hard thresholds (too far, too high, too low)
			pcl::PassThrough<pcl::PointXYZI> pass;
			pass.setInputCloud(cloud);
			pass.setFilterFieldName("x");
			pass.setFilterLimits(0,preditive_region_length);
			pass.filter(*cloud);
			pass.setFilterFieldName("y");
			pass.setFilterLimits(-preditive_region_wide,preditive_region_wide);
			pass.filter(*cloud);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(-5,5); //positive z is down, negative z is up
			pass.filter(*cloud);

			//create segmentation object for fitting a plane to points in the full cloud using RANSAC (assuming the fit plane represents the ground)
			pcl::SACSegmentation<pcl::PointXYZI> seg_plane;
			seg_plane.setOptimizeCoefficients (true); //optional (why is this optional??)
			seg_plane.setModelType (pcl::SACMODEL_PLANE);
			seg_plane.setMethodType (pcl::SAC_RANSAC);
			seg_plane.setMaxIterations (1000); //max iterations for RANSAC
			seg_plane.setDistanceThreshold (0.75); //ground detection threshold parameter
			seg_plane.setInputCloud (cloud); //was raw_cloud

			//segment the points fitted to the plane using ransac
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ()); //what is this? (coefficients for fitted plane?)
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices ()); //what is this? (inliers for points that fit the plane?)
			seg_plane.segment (*inliers, *coefficients);

			//seperate the ground points and the points above the ground (object points)
			pcl::ExtractIndices<pcl::PointXYZI> extract;
			extract.setInputCloud (cloud);
			extract.setIndices (inliers);

			extract.setNegative (false);
			pcl::PointCloud<pcl::PointXYZI>::Ptr ground_filtered (new pcl::PointCloud<pcl::PointXYZI>);
			extract.filter (*ground_filtered);

			extract.setNegative (true);
			pcl::PointCloud<pcl::PointXYZI>::Ptr object_filtered (new pcl::PointCloud<pcl::PointXYZI>);
			extract.filter (*object_filtered);

			//project points on the xy plane, and that is the local hazard map
			pcl::PointCloud<pcl::PointXYZI>::Ptr object_filtered_projection (new pcl::PointCloud<pcl::PointXYZI>);
			*object_filtered_projection = *object_filtered;
			for (int i=0; i<object_filtered->points.size(); i++)
			{
				object_filtered_projection->points[i].z=0;
			}


			int bin_num = 10;
			int bin_counter[bin_num];
			for (int i=0; i<bin_num; i++)
			{
				bin_counter[i] = 0;
			}

			cout << object_filtered_projection->points.size() << endl;
			for(int i=0; i<object_filtered_projection->points.size(); i++)//every point in the predictive region
			{
				bin_counter[(int)ceil((floor(object_filtered_projection->points[i].y) + preditive_region_wide)/(preditive_region_wide*2/bin_num))] += 1; 
			}

			int bin_checker = 0;
			for(int i=0; i<bin_num; i++)
			{
				if(bin_counter[i] == 0)
				{
					bin_checker += 1;
				}
			}

			cout << "Value of bin_checker is " << bin_checker << endl;
			float threshold_ratio = 0.7;
			if(bin_checker <= (int)(bin_num*threshold_ratio)) //less than 70% of the bins are clear
			{
				//return true;//high risk area in front
				cout << "High risk area in front" << endl;

				//trigger the hazard map generation function


			}
			else
			{
				cout << "No risk in front" << endl;
				//return false;
				//relative safe region
			}
			bin_checker = 0; 
		}
		else
		{
			//don't check for collision
			collision=0;
		}
	}
public:
	int collision = 0;
	Registration()
	{
		sub_laser = nn.subscribe("/velodyne_points", 1, &Registration::registrationCallback, this);

	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "collision_detection_node");
	ROS_INFO("collision_detection_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	ros::Publisher pub_col = nh.advertise<messages::CollisionOut>("lidar/collisiondetectionout/collisiondetectionout",1);

	//subscribers
	Registration my_registration;

	//output messages
	messages::CollisionOut msg_CollisionOut;

	while(ros::ok())
	{
		ROS_INFO("*-*-*-*-*-*-*-*-*-*-*-*-*-*");
		//populate message
		msg_CollisionOut.collision = my_registration.collision; 
		msg_CollisionOut.distance_to_collision = 4; 

		//publish message
		pub_col.publish(msg_CollisionOut);

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}




