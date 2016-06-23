//this programe is used for generating keyframe list

//all keyframe information will be in three vectors
//keyframeMaps contains two pointcloud, for icp and verification
//vertex contains all position of keyframe, x, y, heading
//edges contains all transformation matrix between keyframes

//if g2o changes positon of vertex, I should update TkeyTomap !!!!

//g2o information matrix need to change, different weight


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
#include "pcl/kdtree/kdtree_flann.h"	//using kdtree to find k nearest neighbor


//eigen3 library
#include "Eigen/Dense"

//message files
#include "messages/LocalMap.h"

//g2o library
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/types/slam2d/types_slam2d.h"	//type define, vertex and edge

//headfile for testing
#include <pcl/io/pcd_io.h>	//save pcd file
#include <time.h>	// show calculation time

class Keyframe
{
public:
	Keyframe();

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

	//define a struct for keyframe information
	typedef struct Keyframe_Pointcloud
	{
		PointCloud keyframe_icp_cloud;
		PointCloud keyframe_verification_cloud;

	}Keyframe_Pointcloud;

	//define a struct for transformation matrix, include from index and to index
	typedef struct Transformation_Matrix
	{
		matrix4f transformation_matrix;
		int from_index;
		int to_index;
	}Transformation_Matrix;

	//define a struct for position
	typedef struct Position
	{
		float x;
		float y;
		float heading;
	}Position;

	ros::NodeHandle node;

	//publish 
	ros::Publisher keyframePub;

	//subscribe
	ros::Subscriber localmapSub;

	//define icp
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	//define correspondence finder
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;

	//define KdTree for find k nearest neighbor
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	//parameters
	double distance_correspondence;	//maximum distance for finding a correspondence point
	float edge_rectangle_filter;
	double ratio_height_verification;
	int neighbor_number;	//K nearest neighbor to calculate transformation matrix 

	//globe nav_filter data;
	float x_IMU;
	float y_IMU;
	float heading_IMU;


	const double PI = 3.1415926;

	//angle of cone blocking points from robot body (this need to be determined)
	float b_theta; //angle of blocked points (this is constant)

	//globe ICP pointcloud
	// PointCloud refPointCloud;
	// PointCloud readPointCloud;
	// PointCloud keyPointCloud;


	//globe verify pointcloud
	// PointCloud V_refPointCloud;
	// PointCloud V_readPointCloud;
	// PointCloud V_keyPointCloud;

	//globe matrix
	matrix4f TreadToref;
	matrix4f TrefTomap;
	matrix4f TreadTomap;
	matrix4f TkeyTomap;
	matrix4f guessT;


	//lists of all maps
	std::vector<LocalMap_All> LocalMap_All_s;
	std::vector<LocalMap_ICP> LocalMap_ICP_s;
	std::vector<LocalMap_Verification> LocalMap_Verification_s;
	std::vector<Keyframe_Pointcloud> KeyframeMap_s;

	//lists of all tranformation matrix
	std::vector<Transformation_Matrix> TreadToprekey_s;

	//list of vertex in pointcloudXYZ datatype
	PointCloud Vertex_pointcloud;

	//lists of Vertex and Edges for g2o
	std::vector<Position> Vertex;
	std::vector<Transformation_Matrix> Edges;

	//count input index
	int messages_input_index;

	//ICP index
	int ref_index;
	int read_index;
	int key_index;

	//condition for generating a new keyframe
	double threshold_overlap;
	double threshold_mindistance;
	double threshold_maxdistance;

	//condition for doing g2o
	double threshold_g2odistance;
	bool trigger_g2o;
	int maxiteration;

	//definition for publish
	std::vector<PointCloud> Keyframe_Pointcloud_pub;


	//timer definition
	clock_t start, finish;
	double totaltime;

	void getlocalmapcallback(const messages::LocalMap& LocalMapMsgIn);
	// void set_Navfilter_data(float IMU_x, float IMU_y, float IMU_heading);
	ICP_Result ICP_compute(int ICP_ref_index, int ICP_read_index, int PointDataType);	//*************change return type to struct include tranformation matrix, from index, to index, overlap, true of false***********//
	bool outside_rectangle(float rectangle_x, float rectangle_y);		//determine if the point is inside 2.24m rectangle
	PointCloud remove_block_area(float b_x1, float b_y1, float b_heading1, float b_x0, float b_y0, float b_heading0, PointCloud before_remove_points);	//remove points blocks by masks in pre frame
	bool ICP_verification(int ref_v_index, int read_v_index, matrix4f T, double &overlap, int PointDataType);
	double TransformationMatrix_to_angle(matrix4f matrix);
	matrix4f angle_to_TransformationMatrix(double diff_x, double diff_y,double theta);
	bool Do_g2o(std::vector<Position> &Vertex, std::vector<Transformation_Matrix> &Edges);
	void Pcak_Keyframe_message();









};

Keyframe::Keyframe()
{
	//topic initialization
	localmapSub = node.subscribe("/lidar/lidarfilteringnode/localmap", 1, &Keyframe::getlocalmapcallback, this);

	// keyframePub = node.advertise<message::Keyframe>("/slam/keyframe", 1, true); //need to change the message file
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

	//parameters for verification
	ratio_height_verification = 0.01; 

	//parameters for points filter
	edge_rectangle_filter = 2.24; //length of edgs of rectangle filter
	b_theta = 10 * PI / 180; //angle of blocked points (translate to radian)

	//parameters for generating a new keyframe
	threshold_overlap = 0.5; //overlap less than threshold will generate a new keyframe
	threshold_mindistance = 5; //distance between current frame and previous keyframe less than the threshold, it will not generate a new keyframe
	threshold_maxdistance = 20; //distance between current frame and previous keyframe bigger than the threshold, it will generate a new keyframe

	//parameter for k nearest neighbor
	neighbor_number = 5; //find 5 nearest neighbor to build edge, include the search point and the previous keyframe

	//parameter for g2o min distance
	threshold_g2odistance = 8; //if the distance between current keyframe and nearest neighbor keyframe is less than 8 meter, do g2o
	trigger_g2o = false;
	maxiteration = 10; //max iteration for g2o


	//initialize all vectors
	LocalMap_All_s.clear();
	LocalMap_ICP_s.clear();
	LocalMap_Verification_s.clear();
	KeyframeMap_s.clear();
	Vertex_pointcloud.clear();
	Keyframe_Pointcloud_pub.clear();

	TreadToprekey_s.clear();

	Vertex.clear();
	Edges.clear();

	//index for ICP calculation
	messages_input_index = 0;
	ref_index = 0;
	read_index = 0;
	key_index = 0;

	//transformation matrix
	TreadToref = Eigen::Matrix<float, 4, 4>::Identity();
	TrefTomap = Eigen::Matrix<float, 4, 4>::Identity();
	TreadTomap = Eigen::Matrix<float, 4, 4>::Identity();
	TkeyTomap = Eigen::Matrix<float, 4, 4>::Identity();
	guessT = Eigen::Matrix<float, 4, 4>::Identity();


}



void Keyframe::getlocalmapcallback(const messages::LocalMap& LocalMapMsgIn)
{	
	//timer, debug
	start = clock();
	
	if(LocalMapMsgIn.new_data) //if the localmap data is new, update
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

		//!!!make sure the robot is not turning, if the robot is turning, then do nothing
		if(!LocalMap_All_s[messages_input_index].turnFlag)
		{	
			//ref_index will be 0 all time
			read_index = messages_input_index;


			// ROS_INFO_STREAM("Start working......3  " << LocalMap_ICP_s[read_index].ICP_cloudMsgIn.size());
			//do ICP to get transformation matrix from read to reference

			ICP_Result icp_result;
			icp_result = ICP_compute(ref_index, read_index, 0);

			//conditions for generating a new keyframe, overlap, verification, distance
			//calculate distance between current frame and previous keyframe
			double distance;
			Transformation_Matrix TreadToprekey;

			TreadToprekey.transformation_matrix = icp_result.transformation_matrix;
			TreadToprekey.from_index = icp_result.from_index;
			TreadToprekey.to_index = icp_result.to_index;
			TreadToprekey_s.push_back(TreadToprekey);	//save all transformation between frame to previous keyframe in one time find a new keyframe
			distance = sqrt(TreadToprekey.transformation_matrix(0,3)*TreadToprekey.transformation_matrix(0,3) 
				+ TreadToprekey.transformation_matrix(1,3)*TreadToprekey.transformation_matrix(1,3) 
				+ TreadToprekey.transformation_matrix(2,3)*TreadToprekey.transformation_matrix(2,3));
			ROS_INFO_STREAM("Start working......4  " << distance);
			if((distance > threshold_mindistance) && 
				(icp_result.overlap < threshold_overlap || !icp_result.verification_result || distance > threshold_maxdistance))
			{
				Keyframe_Pointcloud keyframe_pointcloud;
				Position position;
				Transformation_Matrix edges;

				key_index = read_index - 1;	//the previous frame will be as a keyframe


				//get position of the keyframe in globe coordinate, map will be the fix frame (0, 0, 0) (x, y, heading)
				TkeyTomap = TreadToprekey_s[TreadToprekey_s.size() - 2].transformation_matrix * TkeyTomap;	//transformation from current keyframe to map 

				

				position.x = TkeyTomap(0,3);
				position.y = TkeyTomap(1,3);
				position.heading = TransformationMatrix_to_angle(TkeyTomap);	//radian

				//store keyframe position into vertex 
				Vertex.push_back(position);
				//store keyframe position into vertex in pointcloudXYZ datatype
				Vertex_pointcloud.push_back(pcl::PointXYZ(position.x, position.y, 0.0));

				//get point cloud of the keyframe, save icp pointcloud and verification pointcloud
				keyframe_pointcloud.keyframe_icp_cloud = LocalMap_ICP_s[key_index].ICP_cloudMsgIn;
				keyframe_pointcloud.keyframe_verification_cloud = LocalMap_Verification_s[key_index].Verification_cloudMsgIn;

				//store keyframe point cloud into keyframe maps
				KeyframeMap_s.push_back(keyframe_pointcloud);

				//get transformation between keyframe and previous keyframe, map is not a keyframe, so the first keyframe will be the keyframe next to map
				if(Vertex.size() > 1)	//if only have one keyframe, we don't have transformation matrix
				{
					edges.transformation_matrix = TreadToprekey_s[TreadToprekey_s.size() - 2].transformation_matrix;
					edges.from_index = Vertex.size() - 1;
					edges.to_index = Vertex.size() - 2;
					//store transformation between current keyframe and previous keyframe into edges
					Edges.push_back(edges);
				}


				//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<find k nearest neighbor>>>>>>>>>>>>>>>>>>>>>>>>>>//

				pcl::PointXYZ searchPoint;	//set the last vertex be the search point
				searchPoint.x = Vertex[Vertex.size() - 1].x;
				searchPoint.y = Vertex[Vertex.size() - 1].y;
				searchPoint.z = 0.0;

				pcl::PointCloud<pcl::PointXYZ>::Ptr Vertex_pointcloud_Ptr (new pcl::PointCloud<pcl::PointXYZ>);
				*Vertex_pointcloud_Ptr = Vertex_pointcloud;	//set the input cloud include the search point and the previous keyframe

				kdtree.setInputCloud(Vertex_pointcloud_Ptr);

				std::vector<int> pointIdxNKNSearch(neighbor_number);
				std::vector<float> pointNKNSquaredDistance(neighbor_number);
				pointIdxNKNSearch.clear();
				pointNKNSquaredDistance.clear();

				if(kdtree.nearestKSearch (searchPoint, neighbor_number, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)	//find nearest neighbor
				{	ROS_INFO_STREAM("Start working......5  " << kdtree.nearestKSearch (searchPoint, neighbor_number, pointIdxNKNSearch, pointNKNSquaredDistance));
					for(int i = 0; i < pointIdxNKNSearch.size(); i++)
					{
						if(pointIdxNKNSearch[i] != Vertex.size() - 1 && pointIdxNKNSearch[i] != Vertex.size() - 2)	//the current keyframe does not need to do ICP with itself and previous keyframe
						{
							//do ICP between k nearest neighbor keyframe to get edges
							icp_result = ICP_compute(pointIdxNKNSearch[i], Vertex.size() - 1, 1);
							ROS_INFO_STREAM("Start working......6  " << "overlap: " << icp_result.overlap << " "<<icp_result.verification_result);

							if(icp_result.verification_result)
							{
								edges.transformation_matrix = icp_result.transformation_matrix;
								edges.from_index = icp_result.from_index;
								edges.to_index = icp_result.to_index;

								//store transformation between keyframes into edges
								Edges.push_back(edges);
								

								//check the distance between current keyframe and nearest neighbor keyframe, and if the distance is less than threshold, do g2o
								if(pointNKNSquaredDistance[i] < threshold_g2odistance)
								{
									distance = sqrt(icp_result.transformation_matrix(0,3)*icp_result.transformation_matrix(0,3) + 
											icp_result.transformation_matrix(1,3)*icp_result.transformation_matrix(1,3) + 
											icp_result.transformation_matrix(2,3)*icp_result.transformation_matrix(2,3));	//using ICP result to verification the distance

									if(distance < threshold_g2odistance)
										trigger_g2o = Do_g2o(Vertex, Edges); //do g2o
								}
							}

						}
					}
				}



				//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<pack keyframe message>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//	
				Pcak_Keyframe_message();


				//after generating a new keyframe, clean all map data for free storage
				//keep all information about the keyframe
				localmap_all = LocalMap_All_s[key_index];
				localmap_icp = LocalMap_ICP_s[key_index];
				localmap_verification = LocalMap_Verification_s[key_index];

				//clean all data
				LocalMap_All_s.clear();
				LocalMap_ICP_s.clear();
				LocalMap_Verification_s.clear();
				TreadToprekey_s.clear();

				//restore the keyframe information as the initial data for next keyframe 
				LocalMap_All_s.push_back(localmap_all);
				LocalMap_ICP_s.push_back(localmap_icp);
				LocalMap_Verification_s.push_back(localmap_verification);

				//reset all index 
				messages_input_index = 0;

			}

			// ROS_INFO_STREAM("trigger_g2o: " << trigger_g2o);
					// if(trigger_g2o = true)
					// 	trigger_g2o = false;
		
			//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//
			//***************************timer*************************//
			finish = clock();
			totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
		
			//*****************************debug***************************//

			// ROS_INFO_STREAM("guessT:  \n" << icp_result.transformation_matrix);
			// ROS_INFO_STREAM("overlap:  " << icp_result.overlap);
			// ROS_INFO_STREAM("verification_result:  " << icp_result.verification_result);
			ROS_INFO_STREAM("time: " << totaltime << "s");

			if(KeyframeMap_s.size() != 0)
			{
				ROS_INFO_STREAM("number of keyframe:  " << KeyframeMap_s.size());
				// ROS_INFO_STREAM("x_key of keyframe:  \n" << KeyframeMap_s[KeyframeMap_s.size()-1].x_key);
			}
	
		}

		

		//when a new local map message come, messages_input_index add one
		messages_input_index++;



	}
	
}

//**************************ICP calculation**********************//
//PointDataType: 0: frame to pre keyframe, 1: keyframe to keyframe
Keyframe::ICP_Result Keyframe::ICP_compute(int ICP_ref_index, int ICP_read_index, int PointDataType)
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
	if(PointDataType == 0)	//data from imu for calculate frame to pre keyframe
	{
		x0 = LocalMap_All_s[ICP_ref_index].x_filter;
		y0 = LocalMap_All_s[ICP_ref_index].y_filter;
		heading0 = LocalMap_All_s[ICP_ref_index].heading_filter;

		x1 = LocalMap_All_s[ICP_read_index].x_filter;
		y1 = LocalMap_All_s[ICP_read_index].y_filter;
		heading1 = LocalMap_All_s[ICP_read_index].heading_filter;
	}
	else if(PointDataType == 1)	//data from keyframe position
	{
		x0 = Vertex[ICP_ref_index].x;
		y0 = Vertex[ICP_ref_index].y;
		heading0 = Vertex[ICP_ref_index].heading;

		x1 = Vertex[ICP_read_index].x;
		y1 = Vertex[ICP_read_index].y;
		heading1 = Vertex[ICP_read_index].heading;
	}

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

	if(PointDataType == 0)	//data from orginal localmap
	{
		readPointCloud_rm = remove_block_area(x1, y1, heading1, x0, y0, heading0, LocalMap_ICP_s[ICP_read_index].ICP_cloudMsgIn);
		refPointCloud_rm = remove_block_area(x0, y0, heading0, x1, y1, heading1, LocalMap_ICP_s[ICP_ref_index].ICP_cloudMsgIn);
	}
	else if(PointDataType == 1)	//data from keyframe
	{
		readPointCloud_rm = remove_block_area(x1, y1, heading1, x0, y0, heading0, KeyframeMap_s[ICP_read_index].keyframe_icp_cloud);
		refPointCloud_rm = remove_block_area(x0, y0, heading0, x1, y1, heading1, KeyframeMap_s[ICP_ref_index].keyframe_icp_cloud);
	}
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
	verification_result = ICP_verification(ICP_ref_index, ICP_read_index, FinalTransformation, overlap, PointDataType);

	icp_result.transformation_matrix = icp.getFinalTransformation();
	icp_result.verification_result = verification_result;
	icp_result.from_index = ICP_read_index;
	icp_result.to_index = ICP_ref_index;
	icp_result.overlap = overlap;

	return icp_result;
}


//************************** points filter **********************//
Keyframe::PointCloud Keyframe::remove_block_area(float b_x1, float b_y1, float b_heading1, float b_x0, float b_y0, float b_heading0, PointCloud before_remove_points)
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
bool Keyframe::outside_rectangle(float rectangle_x, float rectangle_y)
{
	float threshold = edge_rectangle_filter / 2;

	if(rectangle_x < threshold && rectangle_x > -threshold && rectangle_y < threshold && rectangle_y > -threshold)
		return false;
	else
		return true;

}

//********************verification**********************//
bool Keyframe::ICP_verification(int ref_v_index, int read_v_index, matrix4f T, double &overlap, int PointDataType)
{
	//define pointer to get the points information
	PointCloudPtr read_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	PointCloudPtr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr ref_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if(PointDataType == 0)	//data from orginal localmap
	{
  		*read_cloud = LocalMap_Verification_s[read_v_index].Verification_cloudMsgIn;
  		*ref_cloud = LocalMap_Verification_s[ref_v_index].Verification_cloudMsgIn;
	}
	else if(PointDataType == 1)	//data from keyframe
	{
		*read_cloud = KeyframeMap_s[read_v_index].keyframe_verification_cloud;
  		*ref_cloud = KeyframeMap_s[ref_v_index].keyframe_verification_cloud;
	}
	//using transformation matrix transfer read_points to the same coordinate as ref_points
  	pcl::transformPointCloud (*read_cloud, *transformed_cloud, T);

  	//find correspondence
  	est.setInputSource (transformed_cloud);
  	est.setInputTarget (ref_cloud);

  	pcl::Correspondences all_correspondences;
	ROS_INFO_STREAM("Start working......5  " << ref_cloud->size());
	ROS_INFO_STREAM("Start working......6  " << transformed_cloud->size());

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

bool Keyframe::Do_g2o(std::vector<Position> &Vertex, std::vector<Transformation_Matrix> &Edges)
{
	//create the linear solver
	g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();

	//create the block solver on the top of the linear solver
	g2o::BlockSolverX* blockSolver = new g2o::BlockSolverX(linearSolver);

	//create the algorithm to carry out the optimization
	g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

	//create the optimizer
	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm(optimizationAlgorithm);

	//add vertex
	for(int i = 0; i < Vertex.size(); i++)
	{
		g2o::SE2 vertex_se2(Vertex[i].x, Vertex[i].y, Vertex[i].heading);	//vertex (x, y, heading), heading is radian
		g2o::VertexSE2* vertex = new g2o::VertexSE2;
		vertex -> setId(i);
		vertex -> setEstimate(vertex_se2);
		optimizer.addVertex(vertex);
	}

	//add edge
	std::vector<g2o::EdgeSE2*> edges;	//for reading edge data
	edges.clear();
	for(int i = 0; i < Edges.size(); i++)
	{
		double theta = TransformationMatrix_to_angle(Edges[i].transformation_matrix);	//get angle from transformation, angle in radian
		g2o::SE2 edge_se2(Edges[i].transformation_matrix(0,3), Edges[i].transformation_matrix(1,3), theta);	//edge (x, y, theta) 2d transformation matrix
		Eigen::Matrix<double, 3, 3> information_matrix;	
		information_matrix << 400, 0, 0, 0, 10000, 0, 0, 0, 820.702;	//information matrix, inverse of conversion matrix, need to think about this

		g2o::EdgeSE2* edge = new g2o::EdgeSE2;
		edge -> vertices()[0] = optimizer.vertex(Edges[i].from_index);
		edge -> vertices()[1] = optimizer.vertex(Edges[i].to_index);
		edge -> setMeasurement(edge_se2);
		edge -> setInformation(information_matrix);
		optimizer.addEdge(edge);

		edges.push_back(edge);	//save edge pointer for get data from result
	}

	//save all vertex and edges into optimizer_before.g2o
	optimizer.save("optimizer_before.g2o");

	g2o::VertexSE2* firstPose = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(0));	//fix the first position
	firstPose->setFixed(true);

	optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    ROS_INFO_STREAM("Optimizing ...");
    optimizer.optimize(maxiteration);
    ROS_INFO_STREAM("done.");

    //save all vertex and edges into optimizer_after.g2o
	optimizer.save("optimizer_after.g2o");

	//update the new vertex and edges

	for(int i = 0; i < optimizer.vertices().size(); i++)
	{	
		double vertex_afterg2o[3];
		optimizer.vertex(i)->getEstimateData(vertex_afterg2o);

		Vertex[i].x = vertex_afterg2o[0];
		Vertex[i].y = vertex_afterg2o[1];
		Vertex[i].heading = vertex_afterg2o[2];
	}

	for(int i = 0; i < edges.size(); i++)
	{
		double edge_afterg2o[3];
		matrix4f matrix_update;
		edges[i]->getMeasurementData(edge_afterg2o);
		matrix_update = angle_to_TransformationMatrix(edge_afterg2o[0], edge_afterg2o[1], edge_afterg2o[2]);
		Edges[i].transformation_matrix = matrix_update;
	}

	// freeing the graph memory
  	optimizer.clear();

  	// destroy all the singletons
  	g2o::Factory::destroy();
  	g2o::OptimizationAlgorithmFactory::destroy();
  	g2o::HyperGraphActionLibrary::destroy();

  	return true;
}

double Keyframe::TransformationMatrix_to_angle(matrix4f matrix)	//angle in radian
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

Keyframe::matrix4f Keyframe::angle_to_TransformationMatrix(double diff_x, double diff_y,double theta)	//transfer from (x, y, theta) to 4 * 4 transformation matrix
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

void Keyframe::Pcak_Keyframe_message()
{
	//transfer all keyframe points to globe coordination
	//define pointer to get the points information
	PointCloudPtr read_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	PointCloudPtr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//if g2o worked, then update TkeyTomap
	if(trigger_g2o)
	{
		for(int i = 0; i < Vertex.size(); i++)
		{
			*read_cloud = KeyframeMap_s[i].keyframe_verification_cloud;	
			//using transformation matrix transfer read_points to the same coordinate as ref_points
  			pcl::transformPointCloud (*read_cloud, *transformed_cloud, angle_to_TransformationMatrix(Vertex[i].x, Vertex[i].y,Vertex[i].heading));
			Keyframe_Pointcloud_pub[i] = *transformed_cloud;
		}
		TkeyTomap = angle_to_TransformationMatrix(Vertex[Vertex.size() - 1].x, Vertex[Vertex.size() - 1].y,Vertex[Vertex.size() - 1].heading);

		trigger_g2o = false;	//reset trigger_g2o
	}
	else
	{
		//regular update without g2o work
		*read_cloud = KeyframeMap_s[KeyframeMap_s.size() - 1].keyframe_verification_cloud;	
		//using transformation matrix transfer read_points to the same coordinate as ref_points
  		pcl::transformPointCloud (*read_cloud, *transformed_cloud, TkeyTomap);
  				
  		Keyframe_Pointcloud_pub.push_back(*transformed_cloud);
	}

	//publish updated keyframes
	// ......
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Keyframe_node");

	Keyframe keyframe;

	keyframe.Initialization();

	ros::spin();

	return 0;
}