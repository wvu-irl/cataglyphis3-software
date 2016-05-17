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

//using namespace std;
//using namespace pcl;

const float PI = 3.14159265;
const float DEG2RAD = PI/180;
const float RAD2DEG = 180/PI;

float corridor_width = 1.5; // Width of the virtual corridor
float corridor_length = 5.5; // Length of the virtual corridor, THE CLOSEST DECTECTION IS ~2.8 M
float x_shift = 0.0;
float y_shirt = 0.0;
float lidar_height = 0.75; // height of the lidar sensor
float safe_envelope_angle = DEG2RAD*40; //safe envelope angle size
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

	void registrationCallback(pcl::PointCloud<pcl::PointXYZ> const &input_cloud)
	{
		//check for collisions
		if(robot_control.check_for_collisions==1) 
		{
			//function trim laser down to only points in virtual corridor

			//do ground removal on points in virtual corridor
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // The raw point cloud from the LIDAR
			*cloud = input_cloud;

			pcl::PointCloud<pcl::PointXYZ>::Ptr ground_filtered (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr object_filtered (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointIndicesPtr ground (new pcl::PointIndices);

			// pcl::PassThrough<pcl::PointXYZ> pass;
			// pass.setInputCloud(cloud);
			// pass.setFilterFieldName("x");
			// pass.setFilterLimits(x_shift - corridor_width,x_shift + corridor_width); 
			// pass.filter(*cloud);
			// pass.setInputCloud(cloud);
			// pass.setFilterFieldName("y");
			// pass.setFilterLimits(y_shirt + 0.35,y_shirt + corridor_length);
			// pass.filter(*cloud);
			// ROS_INFO("Virtual corridor has %i points.", cloud->points.size());

			
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
			// // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
			// // pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
			// // // Create the segmentation object
			// // pcl::SACSegmentation<pcl::PointXYZ> seg;
			// // // Optional
			// // seg.setOptimizeCoefficients (true);
			// // // Mandatory
			// // seg.setModelType (pcl::SACMODEL_PLANE);
			// // seg.setMethodType (pcl::SAC_RANSAC);
			// // seg.setMaxIterations (1000);
			// // seg.setDistanceThreshold (0.15); //Ground detection threshold parameter
			// // seg.setInputCloud (cloud);
			// // seg.segment (*inliers, *coefficients);
			// // //pcl::ExtractIndices<pcl::PointXYZ> extract;
			// // extract.setInputCloud (cloud);
			// // extract.setIndices (inliers);
			// // extract.setNegative (false);
			// // extract.filter (*ground_filtered);
			// // extract.setNegative (true);
			// // extract.filter (*object_filtered);
			// B: end

			//check for collision
			int collision_point_counter = 0;
			int collision_left_counter = 0;
			int collision_right_counter = 0;
			// FIRST LAYER: SAFE ENVELOPE
			for (int i=0; i<cloud->points.size(); i++)
			{
				//cout << "x,y,z =" << cloud->points[i].x << ", " << cloud->points[i].y << ", " << cloud->points[i].z << endl;
				//CHECK IF POINT IN CORRIDOR
				if(cloud->points[i].x > 0 && cloud->points[i].y < 1.5 && cloud->points[i].y > -1.5 && cloud->points[i].x<4)
				{
					// CHECK IF THE POINT IS OUTSIDE OF THE SAFE ENVELOPE
					if((abs(lidar_height + cloud->points[i].z)/cloud->points[i].x) > tan(safe_envelope_angle))
					{
						// NEED TO AVOID
						collision_point_counter++;
						if(cloud->points[i].y<0)
						{
							collision_left_counter++;
						}
						else
						{
							collision_right_counter++;
						}
					} 
				}
			}

			if(collision_point_counter>10)
			{
				if(collision_left_counter > collision_right_counter)
				{
					collision=1;
					ROS_INFO("LARGEST COLLISION ON RIGHT");
				}
				else
				{
					collision=2;
					ROS_INFO("LARGEST COLLISION ON LEFT");				
				}
			}
			else
			{
				collision=0;
				ROS_INFO("No Collision...");
			}
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




