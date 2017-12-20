/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

//this programe is doing online localization
#include "math.h"
//include ros head file
#include "ros/ros.h"
#include "ros/console.h"

//eigen3 library
#include "Eigen/Dense"

//message files
#include "messages/NavFilterOut.h"
#include "slam/TkeyTomap_msg.h"
#include "messages/SLAMPoseOut.h"

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
	typedef Eigen::Matrix<float, 4, 4> matrix4f;
	typedef Eigen::Matrix<float, 2, 2> matrix2f;
	typedef Eigen::Matrix<float, 2, 1> matrix2f_point;

	//define a struct for navfilterout messages
	typedef struct Navfilterout_All
	{
		float x_filter;
		float y_filter;
		float heading_filter;
	}Navfilterout_All;

	//define a struct for transformation result
	typedef struct Transformation_Result
	{
		matrix4f transformation_matrix;
	}Transformation_Result;

	//define a struct for position, for testing
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
	ros::Subscriber NavfilterSub;
	ros::Subscriber TkeyTomapSub;

	const double PI = 3.1415926;
	double addition;


	//globe matrix
	matrix4f TreadTokey;
	matrix4f TreadTomap;
	matrix4f TkeyTomap;

	//lists of all positon
	std::vector<Navfilterout_All> Navfilterout_All_s;
	// std::vector<double> heading_record;


	//transformation compute index
	int ref_index;
	int read_index;

	//keyframe positon
	float x;
	float y;
	float heading;

	float pre_x;
	float pre_y;
	float pre_heading;

	float x_filter_sub;
	float y_filter_sub;
	float heading_filter_sub;

	bool homing_updated;

	//recored position data
	std::vector<Position> position_data;
	std::vector<Position> position_data_IMU;

	void getnavfilteroutcallback(const messages::NavFilterOut& NavFilterOutMsgIn);
	void getTkeyTomapcallback(const slam::TkeyTomap_msg& TkeyTomapMsgIn);

	// Transformation_Result Transformation_compute(int ICP_ref_index, int ICP_read_index, int option);
	Transformation_Result Transformation_compute(int ICP_ref_index, int ICP_read_index);
	double TransformationMatrix_to_angle(matrix4f matrix);
	matrix4f angle_to_TransformationMatrix(double diff_x, double diff_y,double theta);
	void Coordinate_Normalize(float x0, float y0, float heading0, float x1, float y1, float heading1, double &theta, double &diff_x, double &diff_y);
};




Localization::Localization()
{
	//topic initialization
	NavfilterSub = node.subscribe("navigation/navigationfilterout/navigationfilterout", 1, &Localization::getnavfilteroutcallback, this);
	TkeyTomapSub = node.subscribe("/slam/TkeyTomap_msg", 1, &Localization::getTkeyTomapcallback, this);

	PositionPub = node.advertise<messages::SLAMPoseOut>("/slam/localizationnode/slamposeout", 1, true);
}

void Localization::Initialization()
{




	//initialize all vectors
	Navfilterout_All_s.clear();
	position_data.clear();
	position_data_IMU.clear();
	// heading_record.clear();

	addition = 0;

	//index for ICP calculation
	ref_index = 0;
	read_index = 1;

	x = 0;
	y = 0;
	heading = 0;

	pre_x = 0;
	pre_y = 0;
	pre_heading = 0;

	x_filter_sub = 0;
	y_filter_sub = 0;
	heading_filter_sub = 0;

	homing_updated = false;

	//transformation matrix
	TreadTokey = Eigen::Matrix<float, 4, 4>::Identity();
	TreadTomap = Eigen::Matrix<float, 4, 4>::Identity();
	TkeyTomap = Eigen::Matrix<float, 4, 4>::Identity();

	Navfilterout_All keyframe_all;
	keyframe_all.x_filter = x;
	keyframe_all.y_filter = y;
	keyframe_all.heading_filter = heading;
	Navfilterout_All_s.push_back(keyframe_all);

	Navfilterout_All navfilterout_all; //initialize struct
	navfilterout_all.x_filter = 0;
	navfilterout_all.y_filter = 0;
	navfilterout_all.heading_filter = 0;

	Navfilterout_All_s.push_back(navfilterout_all);


}

//**************************Transformation calculation**********************//
Localization::Transformation_Result Localization::Transformation_compute(int ICP_ref_index, int ICP_read_index)
{
	Transformation_Result transformation_result;
	matrix4f guessT;

	guessT = Eigen::Matrix<float, 4, 4>::Identity();

	//local x, y, heading for calculating guessT
	float x0, y0, heading0;
	float x1, y1, heading1;
	double theta, diff_x, diff_y;

	//if reference frame and read frame are same frame, return indetity matrix
	if(ICP_ref_index == ICP_read_index)
	{
		transformation_result.transformation_matrix = Eigen::Matrix<float, 4, 4>::Identity();
		return transformation_result;
	}

	//data from imu for calculate read frame to reference frame
	x0 = Navfilterout_All_s[ICP_ref_index].x_filter;
	y0 = Navfilterout_All_s[ICP_ref_index].y_filter;
	heading0 = Navfilterout_All_s[ICP_ref_index].heading_filter;

	x1 = Navfilterout_All_s[ICP_read_index].x_filter;
	y1 = Navfilterout_All_s[ICP_read_index].y_filter;
	heading1 = Navfilterout_All_s[ICP_read_index].heading_filter;

	// ROS_INFO_STREAM("x0: " << x0 << "y0: " << y0 << "heading0: " <<heading0);
	//using Nav filter data to calculate guess transformation matrix
	Coordinate_Normalize(x0, y0, heading0, x1, y1, heading1, theta, diff_x, diff_y);

	guessT = angle_to_TransformationMatrix(diff_x, diff_y, theta);

	transformation_result.transformation_matrix = guessT;

	return transformation_result;
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
	//update the position of keyframe
	x = TkeyTomapMsgIn.x;
	y = TkeyTomapMsgIn.y;
	heading = TkeyTomapMsgIn.heading;

	//update the navfilterout of keyframe
	x_filter_sub = TkeyTomapMsgIn.x_filter;
	y_filter_sub = TkeyTomapMsgIn.y_filter;
	heading_filter_sub = TkeyTomapMsgIn.heading_filter;

	homing_updated = TkeyTomapMsgIn.homing_updated;

}

void Localization::getnavfilteroutcallback(const messages::NavFilterOut& NavFilterOutMsgIn)
{

	//debug, for testing
	// Position position;

	messages::SLAMPoseOut slamposeout;
	//check if the keyframe updated or not
	if(pre_x != x || pre_y != y || pre_heading != heading)
	{
		// if(homing_updated)
 	// 	{
 	// 		addition = 0;
 	// 	}

		Navfilterout_All_s[ref_index].x_filter = x_filter_sub;
		Navfilterout_All_s[ref_index].y_filter = y_filter_sub;
		Navfilterout_All_s[ref_index].heading_filter = heading_filter_sub;

		//use the navfilterout of the keyframe as the reference
		//range of globalHeading should from -inf to inf
		slamposeout.globalX = x;
		slamposeout.globalY = y;
		slamposeout.globalHeading = NavFilterOutMsgIn.heading * PI / 180;

		PositionPub.publish(slamposeout);

		//debug, for testing
		// position.x = x_filter_sub;
		// position.y = y_filter_sub;
		// position.heading = heading_filter_sub;
		// position_data.push_back(position);

		// ROS_INFO_STREAM("Position: x:"<< slamposeout.globalX << " y: "<< slamposeout.globalY << " heading: "<<slamposeout.globalHeading);
		// ROS_INFO_STREAM("IMU_Position: x:"<< Navfilterout_All_s[read_index].x_filter << " y: "<< Navfilterout_All_s[read_index].y_filter << " heading: "<<Navfilterout_All_s[read_index].heading_filter * 180 / PI);
	}

	pre_x = x;
	pre_y = y;
	pre_heading = heading;

	//save navfilter as read
	Navfilterout_All_s[read_index].x_filter = NavFilterOutMsgIn.x_position;
	Navfilterout_All_s[read_index].y_filter = NavFilterOutMsgIn.y_position;
	Navfilterout_All_s[read_index].heading_filter = NavFilterOutMsgIn.heading * PI / 180;

	//get transformation matrix from read to reference
	Transformation_Result transformation_result;
	transformation_result = Transformation_compute(ref_index, read_index);
	TreadTokey = transformation_result.transformation_matrix;
	TreadTomap = TkeyTomap * TreadTokey;

	//keep the heading continuous
	// heading_record.push_back(TransformationMatrix_to_angle(TreadTomap) * 180 / PI);

	// if(heading_record.size() > 1)
	// {
	// 	if(abs(heading_record[heading_record.size() - 1] - heading_record[heading_record.size() - 2]) > 180)
	// 	{
	// 		if(heading_record[heading_record.size() - 1] > heading_record[heading_record.size() - 2])
	// 		{
	// 			addition = addition - 360;
	// 		}
	// 		else
	// 		{
	// 			addition = addition + 360;
	// 		}

	// 		double heading_temp = heading_record[heading_record.size() - 1];
	// 		heading_record.clear();
	// 		heading_record.push_back(heading_temp);
	// 	}
	// }



	slamposeout.globalX = TreadTomap(0,3);
	slamposeout.globalY = TreadTomap(1,3);
	// slamposeout.globalHeading = TransformationMatrix_to_angle(TreadTomap) * 180 / PI + addition;
	slamposeout.globalHeading = Navfilterout_All_s[read_index].heading_filter * 180 / PI;	//using nav filter heading, keep continue
	PositionPub.publish(slamposeout);

	//debug, for testing
	// ROS_INFO_STREAM("Position: x:"<< slamposeout.globalX << " y: "<< slamposeout.globalY << " heading: "<<slamposeout.globalHeading);
	// ROS_INFO_STREAM("IMU_Position: x:"<< Navfilterout_All_s[read_index].x_filter << " y: "<< Navfilterout_All_s[read_index].y_filter << " heading: "<<Navfilterout_All_s[read_index].heading_filter * 180 / PI);

	// position.x = TreadTomap(0,3);
	// position.y = TreadTomap(1,3);
	// position.heading = TransformationMatrix_to_angle(TreadTomap);	//radian
	// position_data.push_back(position);

	// Position position_IMU;
	// position_IMU.x = Navfilterout_All_s[read_index].x_filter;
	// position_IMU.y = Navfilterout_All_s[read_index].y_filter;
	// position_IMU.heading = Navfilterout_All_s[read_index].heading_filter;
	// position_data_IMU.push_back(position_IMU);

	//output position data to txt file
	// if(position_data.size() > 1300)
	// {
	// 	std::ofstream outFile;

	// 	outFile.open("test_SLAM.txt");

	// 	for(int i = 1; i < position_data.size(); i++)
	// 	{
	// 	// outFile << position_data[position_data.size() - 1].x << "\t" << position_data[position_data.size() - 1].y << "\n";
	// 		outFile << position_data[i].x << "\t" << position_data[i].y << "\t" <<position_data[i].heading << "\n";
	// 	}
	// 	outFile.close();
	// }

	// if(position_data_IMU.size() > 1300)
	// {
	// 	std::ofstream outFile;

	// 	outFile.open("test_IMU.txt");

	// 	for(int j = 1; j < position_data_IMU.size(); j++)
	// 	{
	// 	// outFile << position_data[position_data.size() - 1].x << "\t" << position_data[position_data.size() - 1].y << "\n";
	// 		outFile << position_data_IMU[j].x << "\t" << position_data_IMU[j].y << "\t" <<position_data_IMU[j].heading <<"\n";
	// 	}
	// 	outFile.close();
	// }

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "localization_node");

	Localization localization;

	localization.Initialization();

	ROS_INFO_STREAM("Localization node running......");

	ros::spin();

	return 0;
}
