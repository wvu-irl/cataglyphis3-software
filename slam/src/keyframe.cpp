//this programe is used for generating keyframe list

#include "math.h"
//include ros head file
#include "ros/ros.h"
#include "ros/console.h"

//PCL library
#include "pcl_ros/point_cloud.h"

//PCL for icp
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


//eigen3 library
#include "Eigen/Dense"

//message files
#include "messages/LocalMap.h"

class Keyframe
{
public:
	Keyframe();

	void Initialization();


protected:
	//define type
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
	typedef Eigen::Matrix<float, 4, 4> Matrix4f;

	// //define a struct for each cell
	// struct cell
	// {
	// 	float x_mean;
	// 	float y_mean;
	// 	float z_mean;
	// 	float var_z;
	// 	bool ground_adjacent;
	// };

	//define a struct for localmap messages, include whole information
	typedef struct LocalMap_All
	{
		std::vector<float> x_mean;
		std::vector<float> y_mean;
		std::vector<float> z_mean;
		std::vector<float> var_z;
		std::vector<bool> ground_adjacent;

		float x_filter;
		float y_filter;
		float heading_filter;
	}LocalMap_All;

	//define a struct for localmap messages, include all central points x, y, for ICP calculation
	typedef struct LocalMap_ICP
	{
		PointCloud ICP_cloudMsgIn;
	}LocalMap_ICP;

	//define a struct for localmap messages. include all information in cells, x, y, z, var_z, for varification
	typedef struct LocalMap_Varification
	{
		PointCloud Varification_cloudMsgIn;
		std::vector<float> z_mean;
		std::vector<float> var_z;
	}LocalMap_Varification;

	ros::NodeHandle node;

	//publish 
	ros::Publisher keyframePub;

	//subscribe
	ros::Subscriber localmapSub;

	//define icp
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	//define correspondence finder
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;

	//parameters
	double distance_correspondence;	//maximum distance for finding a correspondence point
	float edge_rectangle_filter;

	//globe nav_filter data;
	float x_IMU;
	float y_IMU;
	float heading_IMU;

	//local x, y, heading for calculating guessT
	float x0, y0, heading0;
	float x1, y1, heading1;
	double theta, diff_x, diff_y, sin_theta, cos_theta;
	const double PI = 3.1415926;

	//globe ICP pointcloud
	PointCloud refPointCloud;
	PointCloud readPointCloud;
	PointCloud keyPointCloud;
	PointCloudPtr refPointCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr readPointCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);

	//globe verify pointcloud
	PointCloud V_refPointCloud;
	PointCloud V_readPointCloud;
	PointCloud V_keyPointCloud;

	//globe matrix
	Matrix4f TreadToref(4,4);
	Matrix4f TrefTomap(4,4);
	Matrix4f TreadTomap(4,4);
	Matrix4f guessT(4,4);

	//lists of all maps
	std::vector<LocalMap_All> LocalMap_All_s;
	std::vector<LocalMap_ICP> LocalMap_ICP_s;
	std::vector<LocalMap_Varification> LocalMap_Varification_s;

	//count input index
	int message_index;

	void getlocalmapcallback(const message::LocalMap& LocalMapMsgIn);
	void set_Navfilter_data(float IMU_x, float IMU_y, float IMU_heading);
	Matrix4f ICP_compute(PointCloud new_ICP_pointcloud);
	bool outside_rectangle(float rectangle_x, float rectangle_y)		//determine if the point is inside 2.24m rectangle





}

Keyframe::Keyframe()
{
	//topic initialization
	localmapSub = node.subscribe("/lidar/lidarfilteringnode/localmap", 1, &Keyframe::getlocalmapcallback, this);

	keyframePub = node.advertise<message::Keyframe>("/slam/keyframe", 1, true); //need to change the message file
}

void Keyframe::Initialization()
{
	//parameters for icp
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (20);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (1000);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (0.001);

	//parameters for correspondence
	distance_correspondence = 1.4; //maximum distance for finding correspondence points

	//parameters for points filter
	edge_rectangle_filter = 2.24; //length of edgs of rectangle filter

	//initialize all vectors
	LocalMap_All_s.clear();
	LocalMap_ICP_s.clear();
	LocalMap_Varification_s.clear();

	message_index = 0;


}

// void Keyframe::set_Navfilter_data(float IMU_x, float IMU_y, float IMU_heading)
// {
// 	x_IMU = IMU_x;
// 	y_IMU = IMU_y;
// 	heading_IMU = IMU_heading;
// }

// Matrix4f Keyframe::ICP_compute(PointCloud new_ICP_pointcloud, PointCloud new_verify_pointcloud)	//do ICP get transformation matrix, then using height information to verify
// {
// 	//if it its the start point, initialize all parameters
// 	if(refPointCloud.empty() == 1)
// 	{	//ICP data
// 		refPointCloud = new_ICP_pointcloud;
// 		readPointCloud = new_ICP_pointcloud;
// 		keyPointCloud = new_ICP_pointcloud;

// 		TreadToref = Eigen::Matrix<float, 4, 4>::Identity();
// 		TrefTomap = Eigen::Matrix<float, 4, 4>::Identity();
// 		TreadTomap = Eigen::Matrix<float, 4, 4>::Identity();
// 		guessT = Eigen::Matrix<float, 4, 4>::Identity();

// 		x0 = x_IMU;
// 		y0 = y_IMU;
// 		heading0 = heading_IMU;

// 		//verify data
// 		V_refPointCloud = new_verify_pointcloud;
// 		V_readPointCloud = new_verify_pointcloud;
// 		V_keyPointCloud = new_verify_pointcloud;
// 	}
// 	else
// 	//if it is not the start point, update the newest data
// 	{	
// 		//if keyframe update, reset the reference frame as the new keyframe
// 		if(refPointCloud != keyPointCloud)
// 		{	
// 			//ICP data
// 			refPointCloud = keyPointCloud;


// 			x0 = x1;
// 			y0 = y1;
// 			heading0 = heading1;

// 			TrefTomap = TreadTomap;

// 			//verify data
// 			V_refPointCloud = V_keyPointCloud;
// 		}

// 		//ICP data
// 		readPointCloud = new_ICP_pointcloud;

// 		x1 = x_IMU;
// 		y1 = y_IMU;
// 		heading1 = heading_IMU;

// 		//verify data
// 		V_readPointCloud = new_verify_pointcloud;

// 		//using Nav filter data to calculate guess transformation matrix
// 		theta = heading1 - heading0;
// 		theta = theta * PI /180; // degree to radian
// 		diff_x = x1 - x0;
// 		diff_y = y1 - y0;

// 		sin_theta = sin(theta);
// 		cos_theta = cos(theta);

// 		guessT(0,0) = (float)cos_theta;  guessT(0,1) = (float)(-sin_theta);  guessT(0,2) = 0;  guessT(0,3) = (float)diff_x;
// 	 	guessT(1,0) = (float)sin_theta;  guessT(1,1) = (float)cos_theta;     guessT(1,2) = 0;  guessT(1,3) = (float)diff_y;
// 	 	guessT(2,0) = 0;                 guessT(2,1) = 0;					 guessT(2,2) = 1;  guessT(2,3) = 0;
// 	 	guessT(3,0) = 0;                 guessT(3,1) = 0;                    guessT(3,2) = 0;  guessT(3,3) = 1;

// 	}

// 	//icp calculation
// 	refPointCloudPtr = &refPointCloud;
// 	readPointCloudPtr = &readPointCloud;
// 	PointCloud Final;

// 	icp.setInputSource(readPointCloudPtr);
// 	icp.setInputTarget(refPointCloudPtr);
// 	icp.align(Final, guessT);

// 	return icp.getFinalTransformation();
// }

void Keyframe::getlocalmapcallback(const message::LocalMap& LocalMapMsgIn)
{
	if(LocalMapMsgIn.new_data) //if the localmap data is new, update
	{


		//************************************************************//
		//store all information into vector LocalMap_All_s
		LocalMap_All localmap_all; //initialize struct

		localmap_all.x_mean = LocalMapMsgIn.x_mean;
		localmap_all.y_mean = LocalMapMsgIn.y_mean;
		localmap_all.z_mean = LocalMapMsgIn.z_mean;
		localmap_all.var_z = LocalMapMsgIn.var_z;
		localmap_all.x_filter = LocalMapMsgIn.x_filter;
		localmap_all.y_filter = LocalMapMsgIn.y_filter;
		localmap_all.heading_filter = LocalMapMsgIn.heading_filter;
		localmap_all.ground_adjacent = LocalMapMsgIn.ground_adjacent;

		LocalMap_All_s.push_back(localmap_all);


		//************************************************************//
		//store all ground adjacent central points in each localmap into vector LocalMap_ICP_s

		PointCloud ICP_cloudMsgIn; //save ground adjance central points
		ICP_cloudMsgIn.clear();

		PointCloud Varification_cloudMsgIn;	//save all central points
		Varification_cloudMsgIn.clear();

		LocalMap_ICP localmap_icp;

		for(int i = 0; i < LocalMapMsgIn.x_mean.size(); i++)
		{	
			//remove all points inside 2.24m * 2.24m rectangle
			if(outside_rectangle(LocalMapMsgIn.x_mean[i], LocalMapMsgIn.y_mean[i]))
			{
				//if the cell is near to the ground, save that point for icp pointcloud
				if (LocalMapMsgIn.ground_adjacent[i])
				{
					ICP_cloudMSgIn.push_back (pcl::PointXYZ(LocalMapMsgIn.x_mean[i], LocalMapMsgIn.y_mean[i], 0));
				}

				Varification_cloudMsgIn.push_back(pcl::PointXYZ(LocalMapMsgIn.x_mean[i], LocalMapMsgIn.y_mean[i], 0)); // save all central points for varification
			}
		}
		localmap_icp.ICP_cloudMsgIn = ICP_cloudMsgIn;
		LocalMap_ICP_s.push_back(localmap_icp);

		//************************************************************//
		//store all cell information into vector LocalMap_Varification_s

		LocalMap_Varification localmap_varification;

		localmap_varification.Varification_cloudMsgIn = Varification_cloudMsgIn;
		localmap_varification.z_mean = LocalMapMsgIn.z_mean;
		localmap_varification.var_z = LocalMapMsgIn.var_z;

		LocalMap_Varification_s.push_back(LocalMap_Varification);
	
		//when a new local map message come, message_count add one
		message_count++;


	// 	//do ICP to get transformation matrix from read to reference
	// 	TreadToref = ICP_compute(ICP_cloudMSgIn);

	// 	//verify the transformation matrix from ICP using hight information

	}
	
}


//**************************overload "=" ************************//

//************************** points filter **********************//

//************************** circle filter for points ***********//
//determine if the point is inside 2.24m rectangle
bool Keyframe::outside_rectangle(float rectangle_x, float rectangle_y)
{
	float threshold = edge_rectangle_filter / 2;

	if(rectangle_x < threshold && rectangle_x > -threshold && rectangle_y < threshold && rectangle_y > -threshold)
		return false;
	else
		return true;
}