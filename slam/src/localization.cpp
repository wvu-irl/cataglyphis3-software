//this programe is doing online localization
#include "math.h"
//include ros head file
#include "ros/ros.h"
#include "ros/console.h"

//PCL library
#include "pcl_ros/point_cloud.h"

//PCL for icp
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"
#include "pcl/common/transforms.h"	//for transform pointcloud using transformation matrix



//eigen3 library
#include "Eigen/Dense"

//message files
#include "messages/LocalMap.h"
#include "slam/TkeyTomap_msg.h"

class Localization
{
public:
	Localization();

	void Initialization();

protected:
	//define type
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
	typedef Eigen::Matrix<float, 4, 4> matrix4f;

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

		bool turnFlag;
	}LocalMap_All;

	//define a struct for localmap messages, include all central points x, y, for ICP calculation
	typedef struct LocalMap_ICP
	{
		PointCloud ICP_cloudMsgIn;
	}LocalMap_ICP;

	//define a struct for localmap messages. include all information in cells, x, y, z, var_z, for verification
	typedef struct LocalMap_Verification
	{
		PointCloud Verification_cloudMsgIn;
		std::vector<float> z_mean;
		std::vector<float> var_z;
	}LocalMap_Verification;

	//define a struct for ICP result
	typedef struct ICP_Result
	{
		matrix4f transformation_matrix;
		bool verification_result;
		int from_index;
		int to_index;
		double overlap;
	}ICP_Result;

	ros::NodeHandle node;

	//publish 
	ros::Publisher PositionPub;

	//subscribe
	ros::Subscriber localmapSub;
	ros::Subscriber TkeyTomapSub;

	//define icp
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	//define correspondence finder
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;

	//parameters
	double distance_correspondence;	//maximum distance for finding a correspondence point
	float edge_rectangle_filter;
	double ratio_height_verification;

	const double PI = 3.1415926;

	//angle of cone blocking points from robot body (this need to be determined)
	float b_theta; //angle of blocked points (this is constant)

	//globe matrix
	matrix4f TreadToref;
	matrix4f TreadTomap;
	matrix4f TrefTokey;
	matrix4f TkeyTomap;
	matrix4f guessT;


	//lists of all maps
	std::vector<LocalMap_All> LocalMap_All_s;
	std::vector<LocalMap_ICP> LocalMap_ICP_s;
	std::vector<LocalMap_Verification> LocalMap_Verification_s;

	//count input index
	int messages_input_index;

	//ICP index
	int ref_index;
	int read_index;



	//timer definition
	clock_t start, finish;
	double totaltime;

	void getlocalmapcallback(const messages::LocalMap& LocalMapMsgIn);
	void getTkeyTomapcallback(const slam::TkeyTomap_msg& TkeyTomapMsgIn);

	ICP_Result ICP_compute(int ICP_ref_index, int ICP_read_index);	
	bool outside_rectangle(float rectangle_x, float rectangle_y);		//determine if the point is inside 2.24m rectangle
	PointCloud remove_block_area(float b_x1, float b_y1, float b_heading1, float b_x0, float b_y0, float b_heading0, PointCloud before_remove_points);	//remove points blocks by masks in pre frame
	bool ICP_verification(int ref_v_index, int read_v_index, matrix4f T, double &overlap);
	double TransformationMatrix_to_angle(matrix4f matrix);
	matrix4f angle_to_TransformationMatrix(double diff_x, double diff_y,double theta);
};




Localization::Localization()
{
	//topic initialization
	localmapSub = node.subscribe("/lidar/lidarfilteringnode/localmap", 1, &Localization::getlocalmapcallback, this);
	TkeyTomapSub = node.subscribe("/slam/TkeyTomap_msg", 1, &Localization::getTkeyTomapcallback, this);

	// keyframePub = node.advertise<message::Localization>("/slam/keyframe", 1, true); //need to change the message file
}

void Localization::Initialization()
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

	//parameters for verification
	ratio_height_verification = 0.01; 

	//parameters for points filter
	edge_rectangle_filter = 2.24; //length of edgs of rectangle filter
	b_theta = 10 * PI / 180; //angle of blocked points (translate to radian)


	//initialize all vectors
	LocalMap_All_s.clear();
	LocalMap_ICP_s.clear();
	LocalMap_Verification_s.clear();

	//index for ICP calculation
	messages_input_index = 0;
	ref_index = 0;
	read_index = 0;

	//transformation matrix
	TreadToref = Eigen::Matrix<float, 4, 4>::Identity();
	TreadTomap = Eigen::Matrix<float, 4, 4>::Identity();
	TkeyTomap = Eigen::Matrix<float, 4, 4>::Identity();
	guessT = Eigen::Matrix<float, 4, 4>::Identity();
	TrefTokey = Eigen::Matrix<float, 4, 4>::Identity();

}

//**************************ICP calculation**********************//

Localization::ICP_Result Localization::ICP_compute(int ICP_ref_index, int ICP_read_index)
{
	double overlap;
	bool verification_result;
	ICP_Result icp_result;

	PointCloudPtr refPointCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr readPointCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
	
	//local x, y, heading for calculating guessT
	float x0, y0, heading0;
	float x1, y1, heading1;
	double theta, diff_x, diff_y;

	//if reference frame and read frame are same frame, return indetity matrix
	if(ICP_ref_index == ICP_read_index)
	{
		icp_result.transformation_matrix = Eigen::Matrix<float, 4, 4>::Identity();
		icp_result.verification_result = true;
		icp_result.from_index = ICP_read_index;
		icp_result.to_index = ICP_ref_index;
		icp_result.overlap = 1.0;

		return icp_result;
	}
	//data from imu for calculate read frame to reference frame
	x0 = LocalMap_All_s[ICP_ref_index].x_filter;
	y0 = LocalMap_All_s[ICP_ref_index].y_filter;
	heading0 = LocalMap_All_s[ICP_ref_index].heading_filter;

	x1 = LocalMap_All_s[ICP_read_index].x_filter;
	y1 = LocalMap_All_s[ICP_read_index].y_filter;
	heading1 = LocalMap_All_s[ICP_read_index].heading_filter;


	//using Nav filter data to calculate guess transformation matrix
	heading1 = heading1 * PI / 180;	// degree to radian
	heading0 = heading0 * PI / 180;	// degree to radian
	theta = heading1 - heading0; 
	diff_x = x1 - x0;
	diff_y = y1 - y0;

	// sin_theta = sin(theta);
	// cos_theta = cos(theta);

	// Matrix(0,0) = (float)cos_theta;  Matrix(0,1) = (float)(-sin_theta);  Matrix(0,2) = 0;  Matrix(0,3) = (float)diff_x;
	// Matrix(1,0) = (float)sin_theta;  Matrix(1,1) = (float)cos_theta;     Matrix(1,2) = 0;  Matrix(1,3) = (float)diff_y;
	// Matrix(2,0) = 0;                 Matrix(2,1) = 0;					 Matrix(2,2) = 1;  Matrix(2,3) = 0;
	// Matrix(3,0) = 0;                 Matrix(3,1) = 0;                    Matrix(3,2) = 0;  Matrix(3,3) = 1;

	guessT = angle_to_TransformationMatrix(diff_x, diff_y, theta);
	


	//icp calculation

	
	// ROS_INFO_STREAM("Start working......4  " << refPointCloudPtr->size());
	//remove points blocked in ref frame and read frame
	PointCloud readPointCloud_rm;
	PointCloud refPointCloud_rm;

	readPointCloud_rm = remove_block_area(x1, y1, heading1, x0, y0, heading0, LocalMap_ICP_s[ICP_read_index].ICP_cloudMsgIn);
	refPointCloud_rm = remove_block_area(x0, y0, heading0, x1, y1, heading1, LocalMap_ICP_s[ICP_ref_index].ICP_cloudMsgIn);

	*readPointCloudPtr = readPointCloud_rm;
	*refPointCloudPtr = refPointCloud_rm;
	//save pcd for debug
	pcl::io::savePCDFileASCII ("readPointCloud_rm.pcd", readPointCloud_rm);
	pcl::io::savePCDFileASCII ("readPointCloud.pcd", LocalMap_ICP_s[ICP_read_index].ICP_cloudMsgIn);

	pcl::io::savePCDFileASCII ("refPointCloud_rm.pcd", refPointCloud_rm);
	pcl::io::savePCDFileASCII ("refPointCloud.pcd", LocalMap_ICP_s[ICP_ref_index].ICP_cloudMsgIn);

	PointCloud Final;

	icp.setInputSource(readPointCloudPtr);
	icp.setInputTarget(refPointCloudPtr);
	icp.align(Final, guessT);

	matrix4f FinalTransformation;
	FinalTransformation = icp.getFinalTransformation();

	// return icp.getFinalTransformation(); //resutl of ICP
	verification_result = ICP_verification(ICP_ref_index, ICP_read_index, FinalTransformation, overlap);

	icp_result.transformation_matrix = icp.getFinalTransformation();
	icp_result.verification_result = verification_result;
	icp_result.from_index = ICP_read_index;
	icp_result.to_index = ICP_ref_index;
	icp_result.overlap = overlap;

	return icp_result;
}

//************************** points filter **********************//
Localization::PointCloud Localization::remove_block_area(float b_x1, float b_y1, float b_heading1, float b_x0, float b_y0, float b_heading0, PointCloud before_remove_points)
{
	PointCloud after_remove_points;	//define return points
	float phi = 90 * PI/ 180 - b_theta;

	//change in position and heading (calculated from global coordinates
	float rx, ry, rheading;
	rx = b_x0 - b_x1;
	ry = b_y0 - b_y1;
	rheading = b_heading0 - b_heading1;

	//Filter points blocked by robot body in previous position
	float psi_p = rheading+phi; 
	float psi_n = rheading-phi;

	//generate sample point to find the correct side of boundary
	float tempx = rx - (float)sin(rheading);
	float tempy = ry + (float)cos(rheading);

	//check side of boundary 1
	int bp_side, bn_side;
	if(tempy - ((float)tan(psi_p)*(tempx-rx)+(ry)) < 0)
		bp_side = 1;
	else
		bp_side = -1; //equivalent to flipping greater than sign


	//check side of boundary 2
	if(tempy - ((float)tan(psi_n)*(tempx-rx)+(ry)) < 0)
		bn_side = 1;
	else
		bn_side = -1; //equivalent to flipping greater than sign

	for(int i = 0; i < before_remove_points.size(); i++)
	{
		if(bp_side * ( before_remove_points[i].y - ((float)tan(psi_p) * (before_remove_points[i].x - rx) + ry) ) < 0 || 
			bn_side*( before_remove_points[i].y - ((float)tan(psi_n) * (before_remove_points[i].x-rx) + ry) ) < 0)

			after_remove_points.push_back(pcl::PointXYZ(before_remove_points[i].x, before_remove_points[i].y, 0.0));	
	}

	return after_remove_points;

}
//************************** circle filter for points ***********//
//determine if the point is inside 2.24m rectangle
bool Localization::outside_rectangle(float rectangle_x, float rectangle_y)
{
	float threshold = edge_rectangle_filter / 2;

	if(rectangle_x < threshold && rectangle_x > -threshold && rectangle_y < threshold && rectangle_y > -threshold)
		return false;
	else
		return true;

}

//********************verification**********************//
bool Localization::ICP_verification(int ref_v_index, int read_v_index, matrix4f T, double &overlap)
{
	//define pointer to get the points information
	PointCloudPtr read_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	PointCloudPtr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr ref_cloud (new pcl::PointCloud<pcl::PointXYZ>);


  	*read_cloud = LocalMap_Verification_s[read_v_index].Verification_cloudMsgIn;
  	*ref_cloud = LocalMap_Verification_s[ref_v_index].Verification_cloudMsgIn;

	//using transformation matrix transfer read_points to the same coordinate as ref_points
  	pcl::transformPointCloud (*read_cloud, *transformed_cloud, T);

  	//find correspondence
  	est.setInputSource (transformed_cloud);
  	est.setInputTarget (ref_cloud);

  	pcl::Correspondences all_correspondences;

  	est.determineCorrespondences (all_correspondences, distance_correspondence);

  	//calculate overlap (number of overlap points / number of read points)
  	overlap = all_correspondences.size() / (double) transformed_cloud->size();

  	//using height information to do verification
  	float read_z_value;
  	float ref_z_value;
  	float ref_var_z;
  	int count = 0;
  	for(int i = 0; i < all_correspondences.size(); i++)
  	{
  		read_z_value = LocalMap_Verification_s[read_v_index].z_mean[all_correspondences[i].index_query];
  		ref_z_value = LocalMap_Verification_s[ref_v_index].z_mean[all_correspondences[i].index_match];
  		ref_var_z = LocalMap_Verification_s[ref_v_index].var_z[all_correspondences[i].index_match];
  		
  		if(read_z_value < ref_z_value + 3 * ref_var_z && read_z_value > ref_z_value - 3 * ref_var_z)
  			count++;
  	}
  	ROS_INFO_STREAM("height are same: "<< count);
  	ROS_INFO_STREAM("all correspondence: " << all_correspondences.size());

  	if(count / (double) all_correspondences.size() > ratio_height_verification)
  		return true;
  	else
  		return false;
}

double Localization::TransformationMatrix_to_angle(matrix4f matrix)	//angle in radian
{
	double angle = 0.0;	

	if (matrix(0,0) != 0)
		angle = atan2(-matrix(0,1), matrix(0,0));
	else if (matrix(0,1) == 1)
		angle = 0.5 * PI;
	else if (matrix(0,1) == -1)
		angle = -0.5 * PI;

	return angle;
}

Localization::matrix4f Localization::angle_to_TransformationMatrix(double diff_x, double diff_y,double theta)	//transfer from (x, y, theta) to 4 * 4 transformation matrix
{
	matrix4f Matrix;
	double sin_theta, cos_theta;

	sin_theta = sin(theta);
	cos_theta = cos(theta);

	Matrix(0,0) = (float)cos_theta;  Matrix(0,1) = (float)(-sin_theta);  Matrix(0,2) = 0;  Matrix(0,3) = (float)diff_x;
	Matrix(1,0) = (float)sin_theta;  Matrix(1,1) = (float)cos_theta;     Matrix(1,2) = 0;  Matrix(1,3) = (float)diff_y;
	Matrix(2,0) = 0;                 Matrix(2,1) = 0;					 Matrix(2,2) = 1;  Matrix(2,3) = 0;
	Matrix(3,0) = 0;                 Matrix(3,1) = 0;                    Matrix(3,2) = 0;  Matrix(3,3) = 1;

	return Matrix;
}

void Localization::getTkeyTomapcallback(const slam::TkeyTomap_msg& TkeyTomapMsgIn)
{
	TkeyTomap = angle_to_TransformationMatrix(TkeyTomapMsgIn.x, TkeyTomapMsgIn.y, TkeyTomapMsgIn.heading);

}

void Localization::getlocalmapcallback(const messages::LocalMap& LocalMapMsgIn)
{	
	//timer, debug
	start = clock();
	
	if(LocalMapMsgIn.new_data) //if the localmap data is new, update
	{	
		//!!!make sure the robot is not turning, if the robot is turning, then do nothing
		if(!LocalMapMsgIn.turnFlag)
		{
		// ROS_INFO_STREAM("messages_input_index:  " << messages_input_index);
		//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Get data>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

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
		localmap_all.turnFlag = LocalMapMsgIn.turnFlag;
		//localmap_all.ground_adjacent = LocalMapMsgIn.ground_adjacent;

		LocalMap_All_s.push_back(localmap_all);
		// ROS_INFO_STREAM("Start working......1  " << LocalMap_All_s[0].x_mean.size());

		//************************************************************//
		//store all ground adjacent central points in each localmap into vector LocalMap_ICP_s

		PointCloud ICP_cloudMsgIn; //save ground adjance central points
		ICP_cloudMsgIn.clear();

		PointCloud Verification_cloudMsgIn;	//save all central points
		Verification_cloudMsgIn.clear();

		LocalMap_ICP localmap_icp;

		for(int i = 0; i < LocalMapMsgIn.x_mean.size(); i++)
		{	
			//remove all points inside 2.24m * 2.24m rectangle
			if(outside_rectangle(LocalMapMsgIn.x_mean[i], LocalMapMsgIn.y_mean[i]))
			{
				//if the cell is near to the ground, save that point for icp pointcloud
				if (LocalMapMsgIn.ground_adjacent[i])
				{
					ICP_cloudMsgIn.push_back (pcl::PointXYZ(LocalMapMsgIn.x_mean[i], LocalMapMsgIn.y_mean[i], 0));
				}

				Verification_cloudMsgIn.push_back(pcl::PointXYZ(LocalMapMsgIn.x_mean[i], LocalMapMsgIn.y_mean[i], 0)); // save all central points for verification
			}
		}
		localmap_icp.ICP_cloudMsgIn = ICP_cloudMsgIn;
		LocalMap_ICP_s.push_back(localmap_icp);
		//************************************************************//
		//store all cell information into vector LocalMap_Verification_s

		LocalMap_Verification localmap_verification;

		localmap_verification.Verification_cloudMsgIn = Verification_cloudMsgIn;
		localmap_verification.z_mean = LocalMapMsgIn.z_mean;
		localmap_verification.var_z = LocalMapMsgIn.var_z;

		LocalMap_Verification_s.push_back(localmap_verification);
		// ROS_INFO_STREAM("Start working......2  " << LocalMap_Verification_s[0].Verification_cloudMsgIn.size());
	
		//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

		//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Decide ICP frame sequence !!  important>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

	
			
			ref_index = read_index;

			read_index = messages_input_index;

			//do ICP to get transformation matrix from read to reference

			ICP_Result icp_result;
			icp_result = ICP_compute(ref_index, read_index);

			if(icp_result.verification_result)
			{
				TreadToref = icp_result.transformation_matrix;
				
				TrefTokey = TreadToref * TrefTokey;

				TreadTomap = TreadToref * TrefTokey * TkeyTomap;

				double position_x = TreadTomap(0,3);
				double position_y = TreadTomap(1,3);
				double position_heading = TransformationMatrix_to_angle(TreadTomap);	//radian

				ROS_INFO_STREAM("Position: x:"<< position_x << " y: "<< position_y << " heading: "<<position_heading);


			//after generating a new keyframe, clean all map data for free storage
			//keep all information about the keyframe
			localmap_all = LocalMap_All_s[read_index];
			localmap_icp = LocalMap_ICP_s[read_index];
			localmap_verification = LocalMap_Verification_s[read_index];

			//clean all data
			LocalMap_All_s.clear();
			LocalMap_ICP_s.clear();
			LocalMap_Verification_s.clear();

			//restore the keyframe information as the initial data for next keyframe 
			LocalMap_All_s.push_back(localmap_all);
			LocalMap_ICP_s.push_back(localmap_icp);
			LocalMap_Verification_s.push_back(localmap_verification);

			//reset all index 
			messages_input_index = 0;
			ref_index = 0;
			read_index = 0;

			}

			// ROS_INFO_STREAM("trigger_g2o: " << trigger_g2o);
					// if(trigger_g2o = true)
					// 	trigger_g2o = false;
		
			//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
			//***************************timer*************************//
			finish = clock();
			totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
		

		//when a new local map message come, messages_input_index add one
		messages_input_index++;

	}

	}
	
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "localization_node");

	Localization localization;

	localization.Initialization();

	ros::spin();

	return 0;
}