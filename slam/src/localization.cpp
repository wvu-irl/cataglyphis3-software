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

//output position data to txt file
#include <fstream>
#include <iostream>

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
	typedef Eigen::Matrix<float, 3, 3> matrix3f;
	typedef Eigen::Matrix<float, 3, 1> matrix3f_point;
	typedef Eigen::Matrix<float, 2, 2> matrix2f;
	typedef Eigen::Matrix<float, 2, 1> matrix2f_point;

	//define a struct for localmap messages, include whole information
	typedef struct LocalMap_All
	{

		float x_filter;
		float y_filter;
		float heading_filter;
	}LocalMap_All;

	//define a struct for ICP result
	typedef struct ICP_Result
	{
		matrix4f transformation_matrix;
		bool verification_result;
		int from_index;
		int to_index;
		double overlap;
	}ICP_Result;

	//define a struct for position
	typedef struct Position
	{
		double x;
		double y;
		double heading;
	}Position;

	ros::NodeHandle node;

	//publish 
	ros::Publisher PositionPub;

	//subscribe
	ros::Subscriber localmapSub;
	ros::Subscriber TkeyTomapSub;

	const double PI = 3.1415926;


	//globe matrix
	matrix4f TreadTokey;
	matrix4f TreadTomap;
	matrix4f TkeyTomap;
	


	//lists of all maps
	std::vector<LocalMap_All> LocalMap_All_s;


	//ICP index
	int ref_index;
	int read_index;

	//keyframe positon
	float x;
	float y;
	float heading;

	float pre_x;
	float pre_y;
	float pre_heading;



	//timer definition
	clock_t start, finish;
	double totaltime;

	//recored position data
	std::vector<Position> position_data;

	void getlocalmapcallback(const messages::LocalMap& LocalMapMsgIn);
	void getTkeyTomapcallback(const slam::TkeyTomap_msg& TkeyTomapMsgIn);

	// ICP_Result ICP_compute(int ICP_ref_index, int ICP_read_index, int option);	
	ICP_Result ICP_compute(int ICP_ref_index, int ICP_read_index);
	double TransformationMatrix_to_angle(matrix4f matrix);
	matrix4f angle_to_TransformationMatrix(double diff_x, double diff_y,double theta);
	void Coordinate_Normalize(float x0, float y0, float heading0, float x1, float y1, float heading1, double &theta, double &diff_x, double &diff_y);
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




	//initialize all vectors
	LocalMap_All_s.clear();
	position_data.clear();

	//index for ICP calculation
	ref_index = 0;
	read_index = 1;

	x = 0;
	y = 0;
	heading = 0;

	pre_x = 0;
	pre_y = 0;
	pre_heading = 0;

	//transformation matrix
	TreadTokey = Eigen::Matrix<float, 4, 4>::Identity();
	TreadTomap = Eigen::Matrix<float, 4, 4>::Identity();
	TkeyTomap = Eigen::Matrix<float, 4, 4>::Identity();

	LocalMap_All keyframe_all;
	keyframe_all.x_filter = x;
	keyframe_all.y_filter = y;
	keyframe_all.heading_filter = heading;
	LocalMap_All_s.push_back(keyframe_all);

	LocalMap_All localmap_all; //initialize struct	
	localmap_all.x_filter = 0;
	localmap_all.y_filter = 0;
	localmap_all.heading_filter = 0;
	//localmap_all.ground_adjacent = LocalMapMsgIn.ground_adjacent;
	LocalMap_All_s.push_back(localmap_all);
	

}

//**************************ICP calculation**********************//

// Localization::ICP_Result Localization::ICP_compute(int ICP_ref_index, int ICP_read_index, int option)
Localization::ICP_Result Localization::ICP_compute(int ICP_ref_index, int ICP_read_index)
{
	double overlap;
	bool verification_result;
	ICP_Result icp_result;
	matrix4f guessT;

	guessT = Eigen::Matrix<float, 4, 4>::Identity();
	
	//local x, y, heading for calculating guessT
	float x0, y0, heading0;
	float x1, y1, heading1;
	double theta, diff_x, diff_y;

	//if reference frame and read frame are same frame, return indetity matrix
	if(ICP_ref_index == ICP_read_index)
	{
		ROS_INFO_STREAM("start working .....1");
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

	ROS_INFO_STREAM("x0: " << x0 << "y0: " << y0 << "heading0: " <<heading0);
	//using Nav filter data to calculate guess transformation matrix
	Coordinate_Normalize(x0, y0, heading0, x1, y1, heading1, theta, diff_x, diff_y);

	guessT = angle_to_TransformationMatrix(diff_x, diff_y, theta);

	icp_result.transformation_matrix = guessT;
	icp_result.verification_result = true;
	icp_result.from_index = ICP_read_index;
	icp_result.to_index = ICP_ref_index;
	icp_result.overlap = 0.8;

	return icp_result;
}



double Localization::TransformationMatrix_to_angle(matrix4f matrix)	//angle in radian
{
	double angle = 0.0;	

	angle = atan2(-matrix(0,1), matrix(0,0));


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

void Localization::Coordinate_Normalize(float x0, float y0, float heading0, float x1, float y1, float heading1, double &theta, double &diff_x, double &diff_y)
{
	float _x0, _y0; 
	float _x1, _y1;

	double cos0 = cos(heading0);
	double sin0 = sin(heading0);
	matrix2f inverse_T0;
	inverse_T0(0,0) = cos0;		inverse_T0(0,1) = sin0;
	inverse_T0(1,0) = -sin0;	inverse_T0(1,1) = cos0;
	matrix2f_point position0;
	position0(0,0) = x0;
	position0(1,0) = y0;
	matrix2f_point local0;
	local0 = inverse_T0 * position0;

	double cos1 = cos(heading1);
	double sin1 = sin(heading1);
	matrix2f inverse_T1;
	inverse_T1(0,0) = cos1;		inverse_T1(0,1) = sin1;
	inverse_T1(1,0) = -sin1;	inverse_T1(1,1) = cos1;
	matrix2f_point position1;
	position1(0,0) = x1;
	position1(1,0) = y1;
	matrix2f_point local1;
	local1 = inverse_T1 * position1;

	theta = heading1 - heading0; 
	double cos10 = cos(theta);
	double sin10 = sin(theta);
	matrix2f transformation10;
	transformation10(0,0) = cos10;		transformation10(0,1) = -sin10;
	transformation10(1,0) = sin10;		transformation10(1,1) = cos10;
	matrix2f_point local10;
	local10 = transformation10 * local1;

	diff_x = local10(0,0) - local0(0,0);
	diff_y = local10(1,0) - local0(1,0);
}

void Localization::getTkeyTomapcallback(const slam::TkeyTomap_msg& TkeyTomapMsgIn)
{
	TkeyTomap = angle_to_TransformationMatrix(TkeyTomapMsgIn.x, TkeyTomapMsgIn.y, TkeyTomapMsgIn.heading);
	// TkeyTomap = Eigen::Matrix<float, 4, 4>::Identity();
	x = TkeyTomapMsgIn.x;
	y = TkeyTomapMsgIn.y;
	heading = TkeyTomapMsgIn.heading;

}

void Localization::getlocalmapcallback(const messages::LocalMap& LocalMapMsgIn)
{	
	//timer, debug
	// start = clock();
	
	if(LocalMapMsgIn.new_data) //if the localmap data is new, update
	{	


		//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Get data>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

		//************************************************************//

		Position position;

		if(pre_x != x || pre_y != y || pre_heading != heading)
		{
			LocalMap_All_s[ref_index].x_filter = LocalMap_All_s[read_index].x_filter;
			LocalMap_All_s[ref_index].y_filter = LocalMap_All_s[read_index].y_filter;
			LocalMap_All_s[ref_index].heading_filter = LocalMap_All_s[read_index].heading_filter;
			
			position.x = x;
			position.y = y;
			position.heading = heading;
			position_data.push_back(position);

			ROS_INFO_STREAM("Position: x:"<< x << " y: "<< y << " heading: "<<heading * 180 / PI);
			ROS_INFO_STREAM("IMU_Position: x:"<< LocalMap_All_s[read_index].x_filter << " y: "<< LocalMap_All_s[read_index].y_filter << " heading: "<<LocalMap_All_s[read_index].heading_filter * 180 / PI);
		}

		pre_x = x;
		pre_y = y;
		pre_heading = heading;


		LocalMap_All_s[read_index].x_filter = LocalMapMsgIn.x_filter;
		LocalMap_All_s[read_index].y_filter = LocalMapMsgIn.y_filter;
		LocalMap_All_s[read_index].heading_filter = LocalMapMsgIn.heading_filter;
	
		//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

		//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Decide ICP frame sequence !!  important>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

			//do ICP to get transformation matrix from read to reference


			ICP_Result icp_result;
			
			icp_result = ICP_compute(ref_index, read_index);


			TreadTokey = icp_result.transformation_matrix;

			TreadTomap = TkeyTomap * TreadTokey;

				


			double position_x = TreadTomap(0,3);
			double position_y = TreadTomap(1,3);
			double position_heading = TransformationMatrix_to_angle(TreadTomap);	//radian

			ROS_INFO_STREAM("Position: x:"<< position_x << " y: "<< position_y << " heading: "<<position_heading * 180 / PI);
			ROS_INFO_STREAM("IMU_Position: x:"<< LocalMap_All_s[read_index].x_filter << " y: "<< LocalMap_All_s[read_index].y_filter << " heading: "<<LocalMap_All_s[read_index].heading_filter * 180 / PI);
			
			position.x = position_x;
			position.y = position_y;
			position.heading = position_heading;
			position_data.push_back(position);
				
			// Position position_test;
			// position_test.x = LocalMap_All_s[read_index].x_filter;
			// position_test.y = LocalMap_All_s[read_index].y_filter;
			// position_test.heading = LocalMap_All_s[read_index].heading_filter;
			// position_data.push_back(position_test);

			//output position data to txt file
			if(position_data.size() > 1300)
			{
			std::ofstream outFile;

			outFile.open("test.txt");

			for(int j = 1; j < position_data.size(); j++)
			{
			// outFile << position_data[position_data.size() - 1].x << "\t" << position_data[position_data.size() - 1].y << "\n";
				outFile << position_data[j].x << "\t" << position_data[j].y << "\n";
			}
			outFile.close();
			}


			// ROS_INFO_STREAM("trigger_g2o: " << trigger_g2o);
					// if(trigger_g2o = true)
					// 	trigger_g2o = false;
		
			//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
			//***************************timer*************************//
			// finish = clock();
			// totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
		




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