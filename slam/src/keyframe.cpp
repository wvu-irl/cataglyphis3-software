//this programe is used for generating keyframe list

#include "math.h"
#include <array>
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
#include "slam/TkeyTomap_msg.h"
#include "messages/KeyframeList.h"
#include "messages/Keyframe.h"

//g2o library
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/types/slam2d/types_slam2d.h"	//type define, vertex and edge

//grid map library
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "map_layers.h"

//headfile for testing
#include <pcl/io/pcd_io.h>	//save pcd file
#include <time.h>	// show calculation time

//output position data to txt file
#include <fstream>
#include <iostream>

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
	typedef Eigen::Matrix<float, 4, 1> matrix4f_point;
	typedef Eigen::Matrix<float, 3, 3> matrix3f;
	typedef Eigen::Matrix<float, 3, 1> matrix3f_point;
	typedef Eigen::Matrix<double, 3, 3> matrix3d;
	typedef Eigen::Matrix<float, 2, 2> matrix2f;
	typedef Eigen::Matrix<float, 2, 1> matrix2f_point;

	//define a struct for localmap messages, include all information
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
		bool stopFlag;
		bool homing_updatedFlag;
	}LocalMap_All;

	//define a struct for localmap messages, include all central points x, y, and z for ICP calculation
	typedef struct LocalMap_ICP
	{
		PointCloud ICP_cloudMsgIn;
		std::vector<float> z_mean;
		std::vector<float> var_z;
	}LocalMap_ICP;

	//define a struct for localmap messages. include all information in cells, x, y, z, var_z,
	typedef struct LocalMap_Information
	{
		PointCloud Information_cloudMsgIn;
		std::vector<float> z_mean;
		std::vector<float> var_z;
		std::vector<bool> ground_adjacent;
	}LocalMap_Information;

	//define a struct for ICP result
	typedef struct ICP_Result
	{
		matrix4f transformation_matrix;
		double verification_result;
		int from_index;
		int to_index;
		double overlap;
		matrix3f information_matrix;
	}ICP_Result;

	//define a struct for keyframe information
	typedef struct Keyframe_Information
	{
		PointCloud keyframe_icp_cloud;
		PointCloud keyframe_cloud;

		float x;
		float y;
		float heading;

		std::vector<float> z_mean;
		std::vector<float> var_z;

	}Keyframe_Information;

	//define a struct for transformation matrix, include from index and to index
	typedef struct Transformation_Matrix
	{
		matrix4f transformation_matrix;
		int from_index;
		int to_index;
		matrix3f information_matrix;
	}Transformation_Matrix;

	//define a struct for position
	typedef struct Position
	{
		float x;
		float y;
		float heading;
	}Position;

	typedef struct Cell_Local
	{
		float x_mean;
		float y_mean;
		float z_mean;
		float var_z;
		bool ground_adjacent;
	}Cell_Local;

	typedef struct Cell_Global
	{
		float x_mean;
		float y_mean;
		float z_mean;
		float var_z;
		bool ground_adjacent;
		bool occupy; // set to 1 if that cell has been occupied
	}Cell_Global;

	ros::NodeHandle node;

	//publish 
	ros::Publisher keyframePub;
	ros::Publisher TkeyTomapPub;

	//subscribe
	ros::Subscriber localmapSub;

	//define icp
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	//define correspondence finder
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;

	//define KdTree for finding k nearest neighbor
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	//parameters
	double max_correspondence_distance;	//maximum distance for finding a correspondence point
	float radius_round_filter;
	int number_neighbor;	//K nearest neighbor to calculate transformation matrix 
	double threshold_verification_low_limit;	//verification rate smaller than the threshold, using IMU data to get transformation matrix
	double threshold_verification_angle;	//if all points are in the same area, then using IMU data to get transformation matrix

	//global nav_filter data;
	float x_IMU;
	float y_IMU;
	float heading_IMU;


	const double PI = 3.1415926;

	const float LIDAR_TO_ROBOT_X = 0.45;	//from lidar to center of robot

	//angle of cone blocking points from robot body (this need to be determined)
	float b_theta; //angle of blocked points (this is constant)

	//global matrix
	matrix4f TkeyTomap;
	
	//lists of all maps
	std::vector<LocalMap_All> LocalMap_All_s;
	std::vector<LocalMap_ICP> LocalMap_ICP_s;
	std::vector<LocalMap_Information> LocalMap_Information_s;
	std::vector<Keyframe_Information> KeyframeMap_s;

	std::vector<Keyframe_Information> KeyframeMap_temp_s;
	std::vector<LocalMap_Information> LocalMap_Information_temp_s;

	//lists of all transformation matrix
	std::vector<Transformation_Matrix> TreadToprekey_s;

	//list of vertex in pointcloudXYZ datatype
	PointCloud Vertex_pointcloud;
	PointCloud Vertex_pointcloud_temp;

	//lists of Vertex and Edges for g2o
	std::vector<Position> Vertex;
	std::vector<Transformation_Matrix> Edges;

	//lists of Vertex temp and Edges temp 
	std::vector<Position> Vertex_temp;
	std::vector<Transformation_Matrix> Edges_temp;

	//lists of Vertex and Edges for sub g2o
	std::vector<Position> Vertex_sub;
	std::vector<Transformation_Matrix> Edges_sub;

	//count input index
	int messages_input_index;
	int messages_turn_count;
	int messages_stop_count;

	//ICP index
	int ref_index;
	int read_index;
	int key_index;

	//overlap
	int LocalMap_size;
	int threshold_overlap_error;

	//condition for generating a new keyframe
	double threshold_mindistance;

	//parameter for g2o
	int maxiteration;
	int threshold_g2o_min_vertex;

	//condition for ICP
	int threshold_ICP_min_points;

	//definition for temp keyframe
	double threshold_permanent_keyframe_distance;

	//global map definition
	int maplength;
	int cell_resolution;

	Cell_Global GlobalMap[500][500];	//global map size has to be changed here, it will be maplength / cell_resolution
	bool firstscan;
	bool homing_updated;

	//definition for publish
	messages::KeyframeList keyframelist_msg;


	//timer definition, for testing
	clock_t start, finish;
	double totaltime;

	//pcd save counter, for testing
	int pcd_counter_overlap;
	int pcd_counter_globalmap;
	int use_IMU_counter;
	int use_ICP_counter;
	int use_g2o_counter;

	void getlocalmapcallback(const messages::LocalMap& LocalMapMsgIn);
	ICP_Result ICP_compute(int ICP_ref_index, int ICP_read_index, int PointDataType, int option);
	ICP_Result ICP_compute(int ICP_ref_index, int ICP_read_index, int PointDataType, int option, matrix4f transformation_matrix);
	bool outside_area(float area_x, float area_y);		//determine if the point is inside 5m round
	LocalMap_ICP remove_block_area(float b_x1, float b_y1, float b_heading1, float b_x0, float b_y0, float b_heading0, LocalMap_ICP before_remove_localmap_icp);	//remove points blocks by masks in pre frame
	double ICP_verification(std::vector<Cell_Local> read_Correspondences_cells_verify, std::vector<Cell_Local> ref_Correspondences_cells_verify, matrix3f& information_matrix);
	double TransformationMatrix_to_angle(matrix4f matrix);
	matrix4f angle_to_TransformationMatrix(double diff_x, double diff_y,double theta);
	bool Do_g2o(std::vector<Position> &Vertex, std::vector<Transformation_Matrix> &Edges);
	messages::Keyframe Pcak_Keyframe_message(Keyframe_Information keyframe_pointcloud_pub);
	void getOverlap(LocalMap_ICP ICP_localmap_read, LocalMap_ICP ICP_localmap_ref, matrix4f Transformation_matrix, int &overlap_check_counter, std::vector<Cell_Local> &read_Correspondences_cells, std::vector<Cell_Local> &ref_Correspondences_cells);
	void Coordinate_Normalize(float x0, float y0, float heading0, float x1, float y1, float heading1, double &theta, double &diff_x, double &diff_y);
	matrix4f inverse4f(matrix4f Transformation_matrix);
	matrix4f get_g2o_transformation_matrix(std::vector<Position> Vertex_g2o);	//use vertex to calculate transformation matrix
	void Store_Information(const messages::LocalMap& LocalMapMsgIn);
	void getSubVertexEdges();
	void DetectNearestKeyframe();
	void TkeyTomap_Pub(int option);
	void Store_temp_vertex_edges(int option);
	void Vector_Clean();
	void getg2oResult();
	//for testing
	double Update_GlobalMap(LocalMap_Information localmap_information_update, matrix4f transformation_matrix_update);
};

Keyframe::Keyframe()
{
	//topic initialization
	localmapSub = node.subscribe("/lidar/lidarfilteringnode/localmap", 1, &Keyframe::getlocalmapcallback, this);

	keyframePub = node.advertise<messages::KeyframeList>("/slam/keyframesnode/keyframelist", 1, true);
	TkeyTomapPub = node.advertise<slam::TkeyTomap_msg>("/slam/TkeyTomap_msg", 1, true);
}

void Keyframe::Initialization()
{
	//parameters for icp
	// Set the max correspondence distance (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (5);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (1000);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (0.001);

	//parameters for correspondence
	max_correspondence_distance = 1.4; //maximum distance for finding correspondence points

	//parameters for verification
	threshold_verification_low_limit = 3;
	threshold_verification_angle = 15;

	//parameters for points filter
	radius_round_filter = 5; //length of edgs of round filter
	b_theta = 16 * PI / 180; //angle of blocked points (translate to radian)

	//parameters for generating a new keyframe
	threshold_mindistance = 5; //distance between current frame and previous keyframe less than the threshold, it will not generate a new keyframe

	//parmeters for temp keyframe
	threshold_permanent_keyframe_distance = 10; //in the threshold area, if a permanent keyframe exist, then discard the temp keyframe

	//overlap
	LocalMap_size = 120; // localmap size will be 120 + 120
 	threshold_overlap_error = 20;	//to find overlap, enlarge the map size to 160*160

	//parameter for k nearest neighbor
	number_neighbor = 5; //find 5 nearest neighbor to build edge, include the search point and the previous keyframe

	//parameter for g2o
	maxiteration = 10; //max iteration for g2o
	threshold_g2o_min_vertex = 10;	//vertex_sub number bigger than the threshold, than do g2o, avoid there is no enough vertex to do g2o

	//parameter for ICP
	threshold_ICP_min_points = 20; //number of points for ICP should bigger than this threshold

	//parameter for global map
	maplength = 500;	//meter
	cell_resolution = 1;	//meter
	firstscan = true;
	homing_updated = false;
	// ServerSwitch = false;


	//initialize all vectors
	LocalMap_All_s.clear();
	LocalMap_ICP_s.clear();
	LocalMap_Information_s.clear();
	KeyframeMap_s.clear();
	Vertex_pointcloud.clear();

	TreadToprekey_s.clear();

	Vertex.clear();
	Edges.clear();

	Vertex_temp.clear();
	Edges_temp.clear();
	KeyframeMap_temp_s.clear();
	Vertex_pointcloud_temp.clear();
	LocalMap_Information_temp_s.clear();

	Vertex_sub.clear();
	Edges_sub.clear();

	//index for ICP calculation
	messages_input_index = 0;
	messages_turn_count = 0;
	messages_stop_count = 0;
	ref_index = 0;
	read_index = 0;
	key_index = 0;

	//transformation matrix
	TkeyTomap = Eigen::Matrix<float, 4, 4>::Identity();
	// TkeyTomap(0,3) = 1;	//offset, robot start in (1,0)

	//global map initialize
	for(int i=0; i<500; i++)
	{
		for(int j=0; j<500; j++)
		{
			GlobalMap[i][j].x_mean = 0;
			GlobalMap[i][j].y_mean = 0;
			GlobalMap[i][j].z_mean = 0;
			GlobalMap[i][j].var_z = 0;
			GlobalMap[i][j].ground_adjacent = 0;
			GlobalMap[i][j].occupy = 0;
		}
	}

	//counter initilization
	pcd_counter_overlap = 0;
	pcd_counter_globalmap = 0;
	use_IMU_counter = 0;
	use_ICP_counter = 0;
	use_g2o_counter = 0;

}



void Keyframe::getlocalmapcallback(const messages::LocalMap& LocalMapMsgIn)
{	
	//timer, debug
	start = clock();

	if(LocalMapMsgIn.startSLAM)	//waiting for start flag
	{
		if(LocalMapMsgIn.new_data) //if the localmap data is new, update
		{
			ROS_INFO_STREAM("messages_input_index:  " << messages_input_index);
			//get data
			Store_Information(LocalMapMsgIn);

			//first scan update to global map
			if(firstscan)
			{	
				//get navfilter to initialize the first scan
				TkeyTomap = angle_to_TransformationMatrix(LocalMap_All_s[messages_input_index].x_filter, LocalMap_All_s[messages_input_index].y_filter, LocalMap_All_s[messages_input_index].heading_filter);
				Update_GlobalMap(LocalMap_Information_s[messages_input_index], TkeyTomap);
				firstscan = false;

				//add the first scan as the first keyframe
				Position position;

				position.x = TkeyTomap(0,3);
				position.y = TkeyTomap(1,3);
				position.heading = LocalMap_All_s[messages_input_index].heading_filter;	//radian, make sure the number is same as Navfilter

				//store keyframe position into vertex 
				Vertex.push_back(position);
				//store keyframe position into vertex in pointcloudXYZ datatype
				Vertex_pointcloud.push_back(pcl::PointXYZ(position.x, position.y, 0.0));

				//get point cloud of the keyframe, save icp pointcloud and verification pointcloud
				Keyframe_Information keyframe_pointcloud;
				keyframe_pointcloud.keyframe_icp_cloud = LocalMap_ICP_s[0].ICP_cloudMsgIn;
				keyframe_pointcloud.keyframe_cloud = LocalMap_Information_s[0].Information_cloudMsgIn;
				keyframe_pointcloud.x = position.x;
				keyframe_pointcloud.y = position.y;
				keyframe_pointcloud.heading = position.heading;
				keyframe_pointcloud.z_mean = LocalMap_Information_s[0].z_mean;
				keyframe_pointcloud.var_z = LocalMap_Information_s[0].var_z;

				//store keyframe point cloud into keyframe maps
				KeyframeMap_s.push_back(keyframe_pointcloud);

				//publish the first keyframe
				keyframelist_msg.keyframeList.push_back(Pcak_Keyframe_message(KeyframeMap_s[KeyframeMap_s.size() - 1]));
				keyframePub.publish(keyframelist_msg);

				//publish the first keyframe to localization, based on Navfilter data
				TkeyTomap_Pub(0);

			}
			else
			{
				//if the robot is stop, at the end of stop, generate a new temp keyframe
				if(LocalMap_All_s[messages_input_index].stopFlag)
				{
					messages_stop_count++;	//count for finding the last stop
					if(LocalMap_All_s[messages_input_index].homing_updatedFlag)	//detect if doing homing update
					{
						homing_updated = true;
					}

					//doing ICP during stop, since the robot still moving when the stop flag turn to true
					//ref_index will be 0 all time
					read_index = messages_input_index;


					//do ICP to get transformation matrix from read to reference
					ICP_Result icp_result;
					icp_result = ICP_compute(ref_index, read_index, 0, 0);

					Transformation_Matrix TreadToprekey;

					TreadToprekey.transformation_matrix = icp_result.transformation_matrix;
					TreadToprekey.from_index = icp_result.from_index;
					TreadToprekey.to_index = icp_result.to_index;
					TreadToprekey.information_matrix = icp_result.information_matrix;

					TreadToprekey_s.push_back(TreadToprekey);	//save all transformation between frame to previous keyframe for finding a new temp keyframe
					getSubVertexEdges();

				}
				else
				{
					if(messages_stop_count > 0)	//the robot start moving after stop
					{
						
						if(homing_updated)	//if did homing update, reset position using navfilter data
						{
							read_index = messages_input_index;
							key_index = read_index;	

							//reset position using navfilter data
							TkeyTomap = angle_to_TransformationMatrix(LocalMap_All_s[key_index].x_filter, LocalMap_All_s[key_index].y_filter, LocalMap_All_s[key_index].heading_filter);

							Store_temp_vertex_edges(0);

							//if second temp keyframe generated, estimate the previous keyframe is permanent or not
							if(Vertex_temp.size() == 2)
							{
								DetectNearestKeyframe();	
							}

							TkeyTomap_Pub(1);

							//after generating a new keyframe, clean all map data for free storage
							Vector_Clean();

							//reset all index 
							messages_input_index = 0;
							messages_stop_count = 0;
							homing_updated = false;
						}
						//at the end of stop, generate a new temp keyframe
						else
						{
							//ref_index will be 0 all time
							read_index = messages_input_index;
						
							if(Vertex_sub.size() > threshold_g2o_min_vertex)	//make sure after stop, the number of vertex_sub is enough to do g2o
							{
								getg2oResult();				
							}

							if(Vertex_sub.size() != 0)	//if the robot stop flag change to false during stop, it will be wrong
							{
								key_index = read_index - 1;	//the pre frame which should be the last frame when the robot stopped will be as a keyframe

								//get position of the keyframe in global coordinate
								TkeyTomap = TkeyTomap * Edges_sub[Edges_sub.size() - 1].transformation_matrix;

								Store_temp_vertex_edges(1);

								//if second temp keyframe generated, estimate the previous keyframe is permanent or not
								if(Vertex_temp.size() == 2)
								{
									DetectNearestKeyframe();	
								}

								TkeyTomap_Pub(0);									
							}

							//after generating a new keyframe, clean all map data for free storage
							Vector_Clean();

							//reset all index 
							messages_input_index = 0;
							messages_stop_count = 0;
						}
					}
					// the stop flag is flase and the robot did not stop before
					else if(messages_stop_count == 0)
					{
						//if the robot is turning, add a vetex after turn, using IMU data to get transformation matrix
						if(LocalMap_All_s[messages_input_index].turnFlag)
						{
							messages_turn_count++;
						}
						else
						{
							//add the first frame as a keyframe after turn
							if(messages_turn_count > 0)
							{
								//ref_index will be 0 all time
								read_index = messages_input_index;
								
								ICP_Result icp_result;
								icp_result = ICP_compute(ref_index, read_index, 0, 1);	//using IMU data to get transformation matrix

								Transformation_Matrix TreadToprekey;

								TreadToprekey.transformation_matrix = icp_result.transformation_matrix;
								TreadToprekey.from_index = icp_result.from_index;
								TreadToprekey.to_index = icp_result.to_index;
								TreadToprekey.information_matrix = icp_result.information_matrix;
								TreadToprekey_s.push_back(TreadToprekey);	//save all transformation between frame to previous keyframe in one time find a new keyframe

								Keyframe_Information keyframe_pointcloud;
								Position position;
								Transformation_Matrix edges;

								key_index = read_index;	//the current frame will be as a keyframe

								//get position of the keyframe in global coordinate
								TkeyTomap = TkeyTomap * TreadToprekey_s[TreadToprekey_s.size() - 1].transformation_matrix;

								Store_temp_vertex_edges(2);

								//if second temp keyframe generated, estimate the previous keyframe is permanent or not
								if(Vertex_temp.size() == 2)
								{
									DetectNearestKeyframe();	
								}

								TkeyTomap_Pub(0);
								
								//after generating a new keyframe, clean all map data for free storage
								Vector_Clean();

								//reset all index 
								messages_input_index = 0;
								messages_turn_count = 0;
							}
							//the robot move normally
							else if(messages_turn_count == 0)
							{
								//ref_index will be 0 all time
								read_index = messages_input_index;

								//do ICP to get transformation matrix from read to reference

								ICP_Result icp_result;
								icp_result = ICP_compute(ref_index, read_index, 0, 0);

								//calculate distance between current frame and previous keyframe
								double distance;
								Transformation_Matrix TreadToprekey;

								TreadToprekey.transformation_matrix = icp_result.transformation_matrix;
								TreadToprekey.from_index = icp_result.from_index;
								TreadToprekey.to_index = icp_result.to_index;
								TreadToprekey.information_matrix = icp_result.information_matrix;
								TreadToprekey_s.push_back(TreadToprekey);	//save all transformation between frame to previous keyframe in one time find a new keyframe
								distance = sqrt(TreadToprekey.transformation_matrix(0,3)*TreadToprekey.transformation_matrix(0,3) 
												+ TreadToprekey.transformation_matrix(1,3)*TreadToprekey.transformation_matrix(1,3) 
												+ TreadToprekey.transformation_matrix(2,3)*TreadToprekey.transformation_matrix(2,3));

								//if the distance is bigger than threshold_mindistance, generate a new temp keyframe
								if(distance > threshold_mindistance)
								{
									getg2oResult();
									
									Keyframe_Information keyframe_pointcloud;
									Position position;
									Transformation_Matrix edges;

									key_index = read_index - 1;	//the previous frame will be as a keyframe

									//get position of the keyframe in global coordinate
									TkeyTomap = TkeyTomap * Edges_sub[Edges_sub.size() - 1].transformation_matrix;	//transformation from current keyframe to map
									
									Store_temp_vertex_edges(1);

									//if second temp keyframe generated, estimate the previous keyframe is permanent or not
									if(Vertex_temp.size() == 2)
									{
										DetectNearestKeyframe();
									}

									TkeyTomap_Pub(0);

									//after generating a new keyframe, clean all map data for free storage
									Vector_Clean();

									//reset all index 
									messages_input_index = 0;
								}
								//if the distance is smaller than threshold_mindistance, store the frame to sub
								else
								{
									getSubVertexEdges();
								}

							}

						}
					}
				}

			}
			//when a new local map message come, messages_input_index add one
			messages_input_index++;

			//for testing
			std::ofstream outFile;

			outFile.open("test_key.txt");

			for(int j = 0; j < Vertex.size(); j++)
			{
				// outFile << position_data[position_data.size() - 1].x << "\t" << position_data[position_data.size() - 1].y << "\n";
				outFile << Vertex[j].x << "\t" << Vertex[j].y << "\n";
			}
			outFile.close();
					
			//***************************timer*************************//
			finish = clock();
			totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	
			//*****************************debug***************************//

			// ROS_INFO_STREAM("guessT:  \n" << icp_result.transformation_matrix);
			// ROS_INFO_STREAM("overlap:  " << icp_result.overlap);
			// ROS_INFO_STREAM("verification_result:  " << icp_result.verification_result);
			ROS_INFO_STREAM("time: " << totaltime << "s");

			// if(KeyframeMap_s.size() != 0)
			// {
			// 	ROS_INFO_STREAM("number of keyframe:  " << KeyframeMap_s.size());
			// 	// ROS_INFO_STREAM("x_key of keyframe:  \n" << KeyframeMap_s[KeyframeMap_s.size()-1].x_key);
			// }

			ROS_INFO_STREAM("use_ICP_counter: " << use_ICP_counter << " use_IMU_counter: " << use_IMU_counter << " use_g2o_counter: " << use_g2o_counter);


		}
	}

}		

//**************************ICP calculation**********************//
//PointDataType: 0: frame to pre keyframe, 1: keyframe to keyframe
//option: 0: return overlap transformation, 1: return IMU transformation, 2: return full size pointcloud transformation
Keyframe::ICP_Result Keyframe::ICP_compute(int ICP_ref_index, int ICP_read_index, int PointDataType, int option)
{
	double overlap;
	double overlap_IMU;
	double overlap_ICP;
	int overlap_check_counter = 0;
	double verification_result;
	double verification_result_IMU;
	double verification_result_ICP;
	ICP_Result icp_result;
	matrix4f guessT;	//IMU
	matrix3f information_matrix;
	matrix3f information_matrix_IMU;
	matrix3f information_matrix_ICP;

	guessT = Eigen::Matrix<float, 4, 4>::Identity();
	information_matrix = Eigen::Matrix<float, 3, 3>::Identity();
	information_matrix_IMU = Eigen::Matrix<float, 3, 3>::Identity();
	information_matrix_ICP = Eigen::Matrix<float, 3, 3>::Identity();

	PointCloudPtr refPointCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr readPointCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);

	std::vector<Cell_Local> read_Correspondences_cells;
	std::vector<Cell_Local> ref_Correspondences_cells;

	read_Correspondences_cells.clear();
	ref_Correspondences_cells.clear();
	
	//local x, y, heading for calculating guessT
	float x0, y0, heading0;
	float x1, y1, heading1;
	double theta, diff_x, diff_y;

	//if reference frame and read frame are same frame, return indetity matrix
	if(ICP_ref_index == ICP_read_index)
	{
		icp_result.transformation_matrix = Eigen::Matrix<float, 4, 4>::Identity();
		icp_result.verification_result = 1.0;
		icp_result.from_index = ICP_read_index;
		icp_result.to_index = ICP_ref_index;
		icp_result.overlap = 1.0;
		icp_result.information_matrix << 99, 99, 99, 99, 99, 99, 99, 99, 99;

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
	Coordinate_Normalize(x0, y0, heading0, x1, y1, heading1, theta, diff_x, diff_y);

	guessT = angle_to_TransformationMatrix(diff_x, diff_y, theta);

	//icp calculation
	//remove points blocked in read frame
	LocalMap_ICP read_localmap_icp_rm;
	LocalMap_ICP ref_localmap_icp_rm;

	if(PointDataType == 0)	//data from orginal localmap
	{
		read_localmap_icp_rm = remove_block_area(x1, y1, heading1, x0, y0, heading0, LocalMap_ICP_s[ICP_read_index]);
		ref_localmap_icp_rm = remove_block_area(x0, y0, heading0, x1, y1, heading1, LocalMap_ICP_s[ICP_ref_index]);
	}
	else if(PointDataType == 1)	//data from keyframe
	{
		LocalMap_ICP Keyframe_localmap_icp;
		Keyframe_localmap_icp.ICP_cloudMsgIn = KeyframeMap_s[ICP_read_index].keyframe_icp_cloud;
		Keyframe_localmap_icp.z_mean = KeyframeMap_s[ICP_read_index].z_mean;
		Keyframe_localmap_icp.var_z = KeyframeMap_s[ICP_read_index].var_z;

		read_localmap_icp_rm = remove_block_area(x1, y1, heading1, x0, y0, heading0, Keyframe_localmap_icp);

		Keyframe_localmap_icp.ICP_cloudMsgIn = KeyframeMap_s[ICP_ref_index].keyframe_icp_cloud;
		Keyframe_localmap_icp.z_mean = KeyframeMap_s[ICP_ref_index].z_mean;
		Keyframe_localmap_icp.var_z = KeyframeMap_s[ICP_ref_index].var_z;
		ref_localmap_icp_rm = remove_block_area(x0, y0, heading0, x1, y1, heading1, Keyframe_localmap_icp);
	}


	if(option == 0)
	{
		//get overlap using IMU data
		getOverlap(read_localmap_icp_rm, ref_localmap_icp_rm, guessT, overlap_check_counter, read_Correspondences_cells, ref_Correspondences_cells);

		verification_result_IMU = ICP_verification(read_Correspondences_cells, ref_Correspondences_cells, information_matrix_IMU);

		overlap_IMU = overlap_check_counter / (double) read_localmap_icp_rm.ICP_cloudMsgIn.size();
		//save all points as pointcloud
		for(int i = 0; i < read_Correspondences_cells.size(); i++)
		{
			readPointCloudPtr->push_back(pcl::PointXYZ(read_Correspondences_cells[i].x_mean, read_Correspondences_cells[i].y_mean, 0));
		}

		for(int i = 0; i < ref_Correspondences_cells.size(); i++)
		{
			refPointCloudPtr->push_back(pcl::PointXYZ(ref_Correspondences_cells[i].x_mean, ref_Correspondences_cells[i].y_mean, 0));
		}
		
		PointCloud Final;
		
		//set min number of points to do ICP
		if(readPointCloudPtr -> size() < threshold_ICP_min_points || refPointCloudPtr -> size() < threshold_ICP_min_points)
		{
			icp_result.transformation_matrix = guessT;
			icp_result.verification_result = verification_result_IMU;
			icp_result.from_index = ICP_read_index;
			icp_result.to_index = ICP_ref_index;
			icp_result.overlap = overlap_IMU;
			icp_result.information_matrix = 10 * information_matrix_IMU;	//add more weight for IMU result, in case the pointcloud is wrong

			return icp_result;
		}
		else
		{
			icp.setInputSource(readPointCloudPtr);
			icp.setInputTarget(refPointCloudPtr);
			icp.align(Final);

			//for testing
			// ROS_INFO_STREAM("save PCD.........");
			// PointCloudPtr finalPointCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
			// *finalPointCloudPtr = Final;
			
			// std::stringstream readPointCloudPtr_pcd;
			// std::stringstream refPointCloudPtr_pcd;
			// std::stringstream finalPointCloudPtr_pcd;

			// readPointCloudPtr_pcd << "/home/chizhao/test_data/read/readPointCloudPtr" << pcd_counter_overlap << ".pcd";
			// refPointCloudPtr_pcd << "/home/chizhao/test_data/ref/refPointCloudPtr" << pcd_counter_overlap << ".pcd";
			// finalPointCloudPtr_pcd << "/home/chizhao/test_data/final/finalPointCloudPtr" << pcd_counter_overlap << ".pcd";
			// pcl::io::savePCDFileASCII (readPointCloudPtr_pcd.str(), *readPointCloudPtr);
			// pcl::io::savePCDFileASCII (refPointCloudPtr_pcd.str(), *refPointCloudPtr);
			// pcd_counter_overlap++;

			matrix4f FinalTransformation;
			matrix4f ICPTransformation;


			ICPTransformation = icp.getFinalTransformation();

			//keep IMU rotation
			matrix2f Ri;
			matrix2f_point Tg;
			matrix2f_point Tg_new;

			Ri(0,0) = guessT(0,0);
			Ri(1,0) = guessT(1,0);
			Ri(0,1) = guessT(0,1);
			Ri(1,1) = guessT(1,1);

			Tg(0,0) = ICPTransformation(0,3);
			Tg(1,0) = ICPTransformation(1,3);

			Tg_new = Ri * Tg;

			FinalTransformation(0,0) = guessT(0,0);  FinalTransformation(0,1) = guessT(0,1);  	FinalTransformation(0,2) = 0;  FinalTransformation(0,3) = guessT(0,3) + Tg_new(0,0);
			FinalTransformation(1,0) = guessT(1,0);  FinalTransformation(1,1) = guessT(1,1);    FinalTransformation(1,2) = 0;  FinalTransformation(1,3) = guessT(1,3) + Tg_new(1,0);
			FinalTransformation(2,0) = 0;                 FinalTransformation(2,1) = 0;					 FinalTransformation(2,2) = 1;  FinalTransformation(2,3) = 0;
			FinalTransformation(3,0) = 0;                 FinalTransformation(3,1) = 0;                    FinalTransformation(3,2) = 0;  FinalTransformation(3,3) = 1;

			getOverlap(read_localmap_icp_rm, ref_localmap_icp_rm,FinalTransformation, overlap_check_counter, read_Correspondences_cells, ref_Correspondences_cells);

			//get ovelap rate
			//calculate overlap (number of overlap points / number of read points)
		  	overlap_ICP = overlap_check_counter / (double) read_localmap_icp_rm.ICP_cloudMsgIn.size();

			verification_result_ICP = ICP_verification(read_Correspondences_cells, ref_Correspondences_cells, information_matrix_ICP);

			if(verification_result_ICP > verification_result_IMU && verification_result_ICP > threshold_verification_low_limit)
			{

				icp_result.transformation_matrix = FinalTransformation;
				icp_result.verification_result = verification_result_ICP;
				icp_result.from_index = ICP_read_index;
				icp_result.to_index = ICP_ref_index;
				icp_result.overlap = overlap_ICP;
				icp_result.information_matrix = information_matrix_ICP;

				use_ICP_counter++;
			}
			else
			{
				icp_result.transformation_matrix = guessT;
				icp_result.verification_result = verification_result_IMU;
				icp_result.from_index = ICP_read_index;
				icp_result.to_index = ICP_ref_index;
				icp_result.overlap = overlap_IMU;
				if(verification_result_ICP < threshold_verification_low_limit)	//if the pointcloud is wrong, add more weight to IMU edge
				{
					icp_result.information_matrix = 10 * information_matrix_IMU;
				}	
				else
				{
					icp_result.information_matrix = information_matrix_IMU;
					
				}	
				use_IMU_counter++;
				
			}

			return icp_result;
		}
	}
	else if(option == 1)	//using IMU data to get transformation matrix
	{		
		getOverlap(read_localmap_icp_rm, ref_localmap_icp_rm, guessT, overlap_check_counter, read_Correspondences_cells, ref_Correspondences_cells);
		
		//get ovelap rate
		//calculate overlap (number of overlap points / number of read points)
  		overlap = overlap_check_counter / (double) read_localmap_icp_rm.ICP_cloudMsgIn.size();

  		verification_result = ICP_verification(read_Correspondences_cells, ref_Correspondences_cells, information_matrix);
		
		icp_result.transformation_matrix = guessT;
		icp_result.verification_result = verification_result;
		icp_result.from_index = ICP_read_index;
		icp_result.to_index = ICP_ref_index;
		icp_result.overlap = overlap;
		icp_result.information_matrix = information_matrix;

		return icp_result;
	}

	else if(option == 2)
	{
		*readPointCloudPtr = read_localmap_icp_rm.ICP_cloudMsgIn;
		*refPointCloudPtr = ref_localmap_icp_rm.ICP_cloudMsgIn;
		
		PointCloud Final;

		icp.setInputSource(readPointCloudPtr);
		icp.setInputTarget(refPointCloudPtr);
		icp.align(Final);

		matrix4f FinalTransformation;

		FinalTransformation = icp.getFinalTransformation();

		getOverlap(read_localmap_icp_rm, ref_localmap_icp_rm, FinalTransformation, overlap_check_counter, read_Correspondences_cells, ref_Correspondences_cells);

		//get ovelap rate
		//calculate overlap (number of overlap points / number of read points)
	  	overlap = overlap_check_counter / (double) read_localmap_icp_rm.ICP_cloudMsgIn.size();

	  	verification_result = ICP_verification(read_Correspondences_cells, ref_Correspondences_cells, information_matrix);

		icp_result.transformation_matrix = FinalTransformation;
		icp_result.verification_result = verification_result;
		icp_result.from_index = ICP_read_index;
		icp_result.to_index = ICP_ref_index;
		icp_result.overlap = overlap;
		icp_result.information_matrix = information_matrix;

		return icp_result;
	}	
}

//get verification score
Keyframe::ICP_Result Keyframe::ICP_compute(int ICP_ref_index, int ICP_read_index, int PointDataType, int option, matrix4f transformation_matrix)
{
	double overlap;
	int overlap_check_counter = 0;
	double verification_result;
	ICP_Result icp_result;

	matrix3f information_matrix;

	information_matrix = Eigen::Matrix<float, 3, 3>::Identity();

	std::vector<Cell_Local> read_Correspondences_cells;
	std::vector<Cell_Local> ref_Correspondences_cells;

	read_Correspondences_cells.clear();
	ref_Correspondences_cells.clear();
	
	//remove points blocked in read frame
	LocalMap_ICP read_localmap_icp_rm;
	LocalMap_ICP ref_localmap_icp_rm;

	//local x, y, heading for calculating guessT
	float x0, y0, heading0;
	float x1, y1, heading1;
	double theta, diff_x, diff_y;

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

	if(PointDataType == 0)	//data from orginal localmap
	{
		read_localmap_icp_rm = remove_block_area(x1, y1, heading1, x0, y0, heading0, LocalMap_ICP_s[ICP_read_index]);
		ref_localmap_icp_rm = remove_block_area(x0, y0, heading0, x1, y1, heading1, LocalMap_ICP_s[ICP_ref_index]);
	}
	else if(PointDataType == 1)	//data from keyframe
	{
		LocalMap_ICP Keyframe_localmap_icp;
		Keyframe_localmap_icp.ICP_cloudMsgIn = KeyframeMap_s[ICP_read_index].keyframe_icp_cloud;
		Keyframe_localmap_icp.z_mean = KeyframeMap_s[ICP_read_index].z_mean;
		Keyframe_localmap_icp.var_z = KeyframeMap_s[ICP_read_index].var_z;

		read_localmap_icp_rm = remove_block_area(x1, y1, heading1, x0, y0, heading0, Keyframe_localmap_icp);

		Keyframe_localmap_icp.ICP_cloudMsgIn = KeyframeMap_s[ICP_ref_index].keyframe_icp_cloud;
		Keyframe_localmap_icp.z_mean = KeyframeMap_s[ICP_ref_index].z_mean;
		Keyframe_localmap_icp.var_z = KeyframeMap_s[ICP_ref_index].var_z;
		ref_localmap_icp_rm = remove_block_area(x0, y0, heading0, x1, y1, heading1, Keyframe_localmap_icp);
	}

	if(option == 3)
	{
		getOverlap(read_localmap_icp_rm, ref_localmap_icp_rm, transformation_matrix, overlap_check_counter, read_Correspondences_cells, ref_Correspondences_cells);

		//get ovelap rate
		//calculate overlap (number of overlap points / number of read points)
	  	overlap = overlap_check_counter / (double) read_localmap_icp_rm.ICP_cloudMsgIn.size();

	  	//if there is no enough points to do verification
	  	if(read_Correspondences_cells.size() < threshold_ICP_min_points || ref_Correspondences_cells.size() < threshold_ICP_min_points)
	  	{
	  		verification_result = 0.1;
	  	}
	  	else
	  	{
	  		verification_result = ICP_verification(read_Correspondences_cells, ref_Correspondences_cells, information_matrix);
	  	}
	  	
		icp_result.verification_result = verification_result;

		return icp_result;
	}
}

Keyframe::LocalMap_ICP Keyframe::remove_block_area(float b_x1, float b_y1, float b_heading1, float b_x0, float b_y0, float b_heading0, LocalMap_ICP before_remove_localmap_icp)
{
	LocalMap_ICP after_remove_localmap_icp;//define return localmap

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

	for(int i = 0; i < before_remove_localmap_icp.ICP_cloudMsgIn.size(); i++)
	{
		if(bp_side * ( before_remove_localmap_icp.ICP_cloudMsgIn.points[i].y - ((float)tan(psi_p) * (before_remove_localmap_icp.ICP_cloudMsgIn.points[i].x - rx) + ry) ) < 0 || 
			bn_side*( before_remove_localmap_icp.ICP_cloudMsgIn.points[i].y - ((float)tan(psi_n) * (before_remove_localmap_icp.ICP_cloudMsgIn.points[i].x-rx) + ry) ) < 0)
		{
			after_remove_localmap_icp.ICP_cloudMsgIn.push_back(pcl::PointXYZ(before_remove_localmap_icp.ICP_cloudMsgIn.points[i].x, before_remove_localmap_icp.ICP_cloudMsgIn.points[i].y, 0.0));
			after_remove_localmap_icp.z_mean.push_back(before_remove_localmap_icp.z_mean[i]);
			after_remove_localmap_icp.var_z.push_back(before_remove_localmap_icp.var_z[i]);
		}
	}

	return after_remove_localmap_icp;

}

//determine if the point is outside 5m round
bool Keyframe::outside_area(float area_x, float area_y)
{

	if(area_x * area_x + area_y * area_y <= radius_round_filter * radius_round_filter)
		return false;
	else
		return true;
}


double Keyframe::ICP_verification(std::vector<Cell_Local> read_Correspondences_cells_verify, std::vector<Cell_Local> ref_Correspondences_cells_verify, matrix3f& information_matrix)
{
	double score = 0;
	//define pointer to get the points information
	PointCloudPtr read_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr ref_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  	for(int i = 0; i < read_Correspondences_cells_verify.size(); i++)
	{
		read_cloud->push_back(pcl::PointXYZ(read_Correspondences_cells_verify[i].x_mean, read_Correspondences_cells_verify[i].y_mean, 0));
	}

	for(int i = 0; i < ref_Correspondences_cells_verify.size(); i++)
	{
		ref_cloud->push_back(pcl::PointXYZ(ref_Correspondences_cells_verify[i].x_mean, ref_Correspondences_cells_verify[i].y_mean, 0));
	}

	//if there is no enough points, return score for IMU
	if(read_cloud->size() < threshold_ICP_min_points|| ref_cloud->size() < threshold_ICP_min_points)
	{
		//set the information_matrix as a average number
  		information_matrix(0,0) = 5;
  		information_matrix(1,1) = 5;
  		information_matrix(2,2) = 5;
	  	information_matrix(0,1) = 0.0;
	  	information_matrix(0,2) = 0.0;
	  	information_matrix(1,0) = 0.0;
	  	information_matrix(1,2) = 0.0;
	  	information_matrix(2,0) = 0.0;
	  	information_matrix(2,1) = 0.0;

	  	return 10.0;
	}
  	//find correspondence
  	est.setInputSource (read_cloud);
  	est.setInputTarget (ref_cloud);

  	pcl::Correspondences all_correspondences;

  	est.determineCorrespondences (all_correspondences, max_correspondence_distance);

  	//using height information to do verification
  	float read_z_value;
  	float ref_z_value;
  	float ref_var_z;

  	//detect if all points are from one aera
  	std::vector<double> angle_sort;
  	angle_sort.clear();
	for(int i = 0; i < all_correspondences.size(); i++)
	{
		angle_sort.push_back((double)atan2(read_Correspondences_cells_verify[all_correspondences[i].index_query].x_mean, read_Correspondences_cells_verify[all_correspondences[i].index_query].y_mean) * 180 / PI);
	} 

	std::sort(angle_sort.begin(), angle_sort.end());

	if(angle_sort.end() - angle_sort.begin() < threshold_verification_angle) //all points are from one aera
	{
		score = 0.1;
	}
	else 	//check the height information and distance between each correspondences point
	{
	  	for(int i = 0; i < all_correspondences.size(); i++)
	  	{
	  		read_z_value = read_Correspondences_cells_verify[all_correspondences[i].index_query].z_mean;
	  		ref_z_value = ref_Correspondences_cells_verify[all_correspondences[i].index_match].z_mean;
	  		ref_var_z = ref_Correspondences_cells_verify[all_correspondences[i].index_match].var_z;
	  		
	  		if(read_z_value < ref_z_value + 1 * ref_var_z && read_z_value > ref_z_value - 1 * ref_var_z)
	  		{

	  			double distance = sqrt((read_Correspondences_cells_verify[all_correspondences[i].index_query].x_mean - ref_Correspondences_cells_verify[all_correspondences[i].index_match].x_mean) *
	  									(read_Correspondences_cells_verify[all_correspondences[i].index_query].x_mean - ref_Correspondences_cells_verify[all_correspondences[i].index_match].x_mean) +
	  									(read_Correspondences_cells_verify[all_correspondences[i].index_query].y_mean - ref_Correspondences_cells_verify[all_correspondences[i].index_match].y_mean) *
	  									(read_Correspondences_cells_verify[all_correspondences[i].index_query].y_mean - ref_Correspondences_cells_verify[all_correspondences[i].index_match].y_mean) );

	  			if(distance == 0)
	  			{
	  				distance = 0.000000000000001;
	  			}
	  			score = score + max_correspondence_distance / distance;
	  		}
	  	}
	}

  	//calculate information matrix for each edge, calculate from inverse of covariance
  	float diff_x;
  	float diff_y;
  	float diff_z;

  	PointCloud diff_points;

  	matrix3f covariance_matrix;
  	for(int i = 0; i < all_correspondences.size(); i++)
  	{
  		diff_x = read_Correspondences_cells_verify[all_correspondences[i].index_query].x_mean - ref_Correspondences_cells_verify[all_correspondences[i].index_match].x_mean;
  		diff_y = read_Correspondences_cells_verify[all_correspondences[i].index_query].y_mean - ref_Correspondences_cells_verify[all_correspondences[i].index_match].y_mean;
  		diff_z = read_Correspondences_cells_verify[all_correspondences[i].index_query].z_mean - ref_Correspondences_cells_verify[all_correspondences[i].index_match].z_mean;

  		diff_points.push_back(pcl::PointXYZ(diff_x, diff_y, diff_z));
  	}

  	pcl::computeCovarianceMatrix (diff_points, covariance_matrix);

  	//make sure the range of each element in information matrix is between -500 and 500
  	if(covariance_matrix(0,0) > 0.002)
  	{
  		information_matrix(0,0) = 1.0 / covariance_matrix(0,0);
  	}
  	else if(covariance_matrix(0,0) >= 0)
  	{
  		information_matrix(0,0) = 500;
  	}
  	else if(covariance_matrix(0,0) < 0)
  	{
  		information_matrix(0,0) = 0.1;
  	}

  	if(covariance_matrix(1,1) > 0.002)
  	{
  		information_matrix(1,1) = 1.0 / covariance_matrix(1,1);
  	}
  	else if(covariance_matrix(1,1) >= 0)
  	{
  		information_matrix(1,1) = 500;
  	}
  	else if(covariance_matrix(1,1) < 0)
  	{
  		information_matrix(1,1) = 0.1;
  	}

  	if(covariance_matrix(2,2) > 0.002)
  	{
  		information_matrix(2,2) = 1.0 / covariance_matrix(2,2);
  	}
  	else if(covariance_matrix(2,2) >= 0)
  	{
  		information_matrix(2,2) = 500;
  	}
  	else if(covariance_matrix(2,2) < 0)
  	{
  		information_matrix(2,2) = 0.1;
  	}

  	information_matrix(0,1) = 0.0;
  	information_matrix(0,2) = 0.0;
  	information_matrix(1,0) = 0.0;
  	information_matrix(1,2) = 0.0;
  	information_matrix(2,0) = 0.0;
  	information_matrix(2,1) = 0.0;

  	return score / (double) read_Correspondences_cells_verify.size();
}


bool Keyframe::Do_g2o(std::vector<Position> &Vertex, std::vector<Transformation_Matrix> &Edges)		//from and to flip
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
		
		//information matrix, inverse of conversion matrix, type is float for g2o
		information_matrix(0,0) = (double) Edges[i].information_matrix(0,0);
		information_matrix(0,1) = (double) Edges[i].information_matrix(0,1);
		information_matrix(0,2) = (double) Edges[i].information_matrix(0,2);
		information_matrix(1,0) = (double) Edges[i].information_matrix(1,0);
		information_matrix(1,1) = (double) Edges[i].information_matrix(1,1);
		information_matrix(1,2) = (double) Edges[i].information_matrix(1,2);
		information_matrix(2,0) = (double) Edges[i].information_matrix(2,0);
		information_matrix(2,1) = (double) Edges[i].information_matrix(2,1);
		information_matrix(2,2) = (double) Edges[i].information_matrix(2,2);

		g2o::EdgeSE2* edge = new g2o::EdgeSE2;
		edge -> vertices()[0] = optimizer.vertex(Edges[i].to_index);
		edge -> vertices()[1] = optimizer.vertex(Edges[i].from_index);
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
  	return true;
}

double Keyframe::TransformationMatrix_to_angle(matrix4f matrix)	//angle in radian
{

	double angle = 0.0;	
	angle = atan2(-matrix(0,1), matrix(0,0));
	return angle;
}

Keyframe::matrix4f Keyframe::angle_to_TransformationMatrix(double diff_x, double diff_y,double theta)	//transfer from (x, y, theta) to 4 * 4 transformation matrix
{
	matrix4f Matrix;
	double sin_theta, cos_theta;

	sin_theta = sin(theta);
	cos_theta = cos(theta);

	Matrix(0,0) = cos_theta;  Matrix(0,1) = (-sin_theta);  Matrix(0,2) = 0;  Matrix(0,3) = diff_x;
	Matrix(1,0) = sin_theta;  Matrix(1,1) = cos_theta;     Matrix(1,2) = 0;  Matrix(1,3) = diff_y;
	Matrix(2,0) = 0;          Matrix(2,1) = 0;			   Matrix(2,2) = 1;  Matrix(2,3) = 0;
	Matrix(3,0) = 0;          Matrix(3,1) = 0;             Matrix(3,2) = 0;  Matrix(3,3) = 1;

	return Matrix;
}


messages::Keyframe Keyframe::Pcak_Keyframe_message(Keyframe_Information keyframe_pointcloud_pub)
{
	messages::Keyframe keyframe_msg;
	grid_map::GridMap keyframe_map;
	grid_map::Length keyframe_map_size;
	const float keyframe_map_resolution = 1.0; // meter
	grid_map::Position keyframe_map_origin;
	grid_map_msgs::GridMap KeyframeMapMsg;

	keyframe_map_origin[0] = 0.0;
	keyframe_map_origin[1] = 0.0;

	keyframe_map_size[0] = LocalMap_size;
	keyframe_map_size[1] = LocalMap_size;

	keyframe_map.setFrameId("map");

	keyframe_map.setGeometry(keyframe_map_size, keyframe_map_resolution, keyframe_map_origin);

	keyframe_map.add(layerToString(_keyframeDriveability), 0);

	//pointcloud inside
	for(int i = 0; i < keyframe_pointcloud_pub.keyframe_cloud.size(); i++)
	{
		keyframe_map.atPosition(layerToString(_keyframeDriveability), grid_map::Position(keyframe_pointcloud_pub.keyframe_cloud.points[i].x, keyframe_pointcloud_pub.keyframe_cloud.points[i].y)) = 2;
	}

	grid_map::GridMapRosConverter::toMessage(keyframe_map, KeyframeMapMsg);

	keyframe_msg.map = KeyframeMapMsg;
	keyframe_msg.x = keyframe_pointcloud_pub.x;
	keyframe_msg.y = keyframe_pointcloud_pub.y;
	keyframe_msg.heading = keyframe_pointcloud_pub.heading;
	keyframe_msg.associatedROI = -1;

	return keyframe_msg;
}

void Keyframe::getOverlap(LocalMap_ICP ICP_localmap_read, LocalMap_ICP ICP_localmap_ref, matrix4f Transformation_matrix, int &overlap_check_counter, std::vector<Cell_Local> &read_Correspondences_cells, std::vector<Cell_Local> &ref_Correspondences_cells)
{
	read_Correspondences_cells.clear();
	ref_Correspondences_cells.clear();
	overlap_check_counter = 0;

	PointCloudPtr transformed_cloud_read (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr transformed_cloud_ref (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr read_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr ref_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	*read_cloud = ICP_localmap_read.ICP_cloudMsgIn;
	*ref_cloud = ICP_localmap_ref.ICP_cloudMsgIn;

	matrix4f inverse_Transformation_matrix;
	inverse_Transformation_matrix = inverse4f(Transformation_matrix);

	//using transformation matrix transfer read_points to the same coordinate as ref_points
  	pcl::transformPointCloud (*read_cloud, *transformed_cloud_read, Transformation_matrix);
  	pcl::transformPointCloud (*ref_cloud, *transformed_cloud_ref, inverse_Transformation_matrix);

  	//transfer overlap points cells to global map coordination
  	matrix4f TreadTomap_IMU;
  	TreadTomap_IMU = Eigen::Matrix<float, 4, 4>::Identity();
  	TreadTomap_IMU = Transformation_matrix;

  	Cell_Local cell_local_read;
  	Cell_Local cell_local_ref;
  	
  	//for read data
  	for(int i = 0; i < transformed_cloud_read->size(); i++)
  	{
  		if(transformed_cloud_read->points[i].x >= (-LocalMap_size / 2 - threshold_overlap_error) 
  			&& transformed_cloud_read->points[i].x <= (LocalMap_size / 2 + threshold_overlap_error)
  			&& transformed_cloud_read->points[i].y >= (-LocalMap_size / 2 - threshold_overlap_error) 
  			&& transformed_cloud_read->points[i].y <= (LocalMap_size / 2 + threshold_overlap_error))		//check map size = localmap size + 2 * error
  		{
  			matrix4f_point pointXY_local_read;
  			matrix4f_point pointXY_global_read;

  			pointXY_local_read(0,0) = read_cloud->points[i].x;
  			pointXY_local_read(1,0) = read_cloud->points[i].y;
  			pointXY_local_read(2,0) = 0;
  			pointXY_local_read(3,0) = 1;

  			pointXY_global_read = TreadTomap_IMU * pointXY_local_read;

  			pointXY_global_read(0,0) = pointXY_global_read(0,0) / pointXY_global_read(3,0);
  			pointXY_global_read(1,0) = pointXY_global_read(1,0) / pointXY_global_read(3,0);

  			cell_local_read.x_mean = pointXY_global_read(0,0);
  			cell_local_read.y_mean = pointXY_global_read(1,0);

  			cell_local_read.z_mean = ICP_localmap_read.z_mean[i];
  			cell_local_read.var_z = ICP_localmap_read.var_z[i];
  			cell_local_read.ground_adjacent = true;
  			read_Correspondences_cells.push_back(cell_local_read);

  			overlap_check_counter++;
  		}
  	}

  	//for ref data
  	for(int i = 0; i < transformed_cloud_ref->size(); i++)
  	{
  		if(transformed_cloud_ref->points[i].x >= (-LocalMap_size / 2 - threshold_overlap_error) 
  			&& transformed_cloud_ref->points[i].x <= (LocalMap_size / 2 + threshold_overlap_error)
  			&& transformed_cloud_ref->points[i].y >= (-LocalMap_size / 2 - threshold_overlap_error) 
  			&& transformed_cloud_ref->points[i].y <= (LocalMap_size / 2 + threshold_overlap_error))		//check map size = localmap size + 2 * error
  		{
  			matrix4f_point pointXY_local_ref;
  			matrix4f_point pointXY_global_ref;

  			pointXY_local_ref(0,0) = ref_cloud->points[i].x;
  			pointXY_local_ref(1,0) = ref_cloud->points[i].y;
  			pointXY_local_ref(2,0) = 0;
  			pointXY_local_ref(3,0) = 1;

  			pointXY_global_ref = pointXY_local_ref;

  			pointXY_global_ref(0,0) = pointXY_global_ref(0,0) / pointXY_global_ref(3,0);
  			pointXY_global_ref(1,0) = pointXY_global_ref(1,0) / pointXY_global_ref(3,0);

  			cell_local_ref.x_mean = pointXY_global_ref(0,0);
  			cell_local_ref.y_mean = pointXY_global_ref(1,0);

  			cell_local_ref.z_mean = ICP_localmap_ref.z_mean[i];
  			cell_local_ref.var_z = ICP_localmap_ref.var_z[i];
  			cell_local_ref.ground_adjacent = true;
  			ref_Correspondences_cells.push_back(cell_local_ref);
  		}
  	}
}

//for testing
double Keyframe::Update_GlobalMap(LocalMap_Information localmap_information_update, matrix4f transformation_matrix_update)
{
	Cell_Global cell_global;
	int update_check_counter = 0;
	double update_rate;

  	for(int i = 0; i < localmap_information_update.Information_cloudMsgIn.size(); i++)
  	{
  		matrix4f_point pointXY_before;
  		matrix4f_point pointXY_after;

  		pointXY_before(0,0) = localmap_information_update.Information_cloudMsgIn.points[i].x;
  		pointXY_before(1,0) = localmap_information_update.Information_cloudMsgIn.points[i].y;
  		pointXY_before(2,0) = 0;
  		pointXY_before(3,0) = 1;

  		pointXY_after = transformation_matrix_update * pointXY_before;

  		pointXY_after(0,0) = pointXY_after(0,0) / pointXY_after(3,0);
  		pointXY_after(1,0) = pointXY_after(1,0) / pointXY_after(3,0);

  		int horizontal_index = 0;
  		int vertical_index = 0;
  		horizontal_index = floor((pointXY_after(0,0) + maplength / 2) / cell_resolution);
  		vertical_index = floor((pointXY_after(1,0) + maplength / 2) / cell_resolution);

  		if(horizontal_index <= maplength && horizontal_index >= 0 && vertical_index <= maplength && vertical_index >= 0)
  		{
	  		if(GlobalMap[horizontal_index][vertical_index].occupy != 1)	// check if that particular cell in the Cell_global has been occupied
	  		{
	  			cell_global.x_mean = pointXY_after(0,0);
	  			cell_global.y_mean = pointXY_after(1,0);
	  			cell_global.z_mean = localmap_information_update.z_mean[i];
	  			cell_global.var_z = localmap_information_update.var_z[i];
	  			cell_global.ground_adjacent = localmap_information_update.ground_adjacent[i];
	  			cell_global.occupy = 1;

	  			GlobalMap[horizontal_index][vertical_index] = cell_global;

	  			update_check_counter++;
	  		}
  		}	
  	}

  	update_rate = update_check_counter / (double)localmap_information_update.Information_cloudMsgIn.size();

  	PointCloudPtr globalMap (new pcl::PointCloud<pcl::PointXYZ>);
  	int counter = 0;
  	for(int i = 0; i< 500; i++)
  	{
  		for(int j = 0; j< 500; j++)
  		{
  			if(GlobalMap[i][j].occupy == 1)
  			{
  				globalMap->push_back(pcl::PointXYZ(GlobalMap[i][j].x_mean, GlobalMap[i][j].y_mean, 0));
  				counter++;
  			}
  		}
  	}

  	//for testing
  	// std::stringstream globalMap_pcd;
  	// globalMap_pcd << "/home/atlas/test_data/globalMap/globalMap" << pcd_counter_globalmap << ".pcd";
  	// pcl::io::savePCDFileASCII (globalMap_pcd.str(), *globalMap);
  	// pcd_counter_globalmap++;

  	return update_rate;
}

void Keyframe::Coordinate_Normalize(float x0, float y0, float heading0, float x1, float y1, float heading1, double &theta, double &diff_x, double &diff_y)
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

Keyframe::matrix4f Keyframe::inverse4f(matrix4f Transformation_matrix)
{
	matrix4f inverseT;

	inverseT(0,0) = Transformation_matrix(0,0);
	inverseT(0,1) = - Transformation_matrix(0,1);
	inverseT(0,2) = 0;
	inverseT(0,3) = - Transformation_matrix(0,3) * Transformation_matrix(0,0) - Transformation_matrix(1,3) * Transformation_matrix(1,0);
	inverseT(1,0) = - Transformation_matrix(1,0);
	inverseT(1,1) = Transformation_matrix(1,1);
	inverseT(1,2) = 0;
	inverseT(1,3) = Transformation_matrix(0,3) * Transformation_matrix(1,0) - Transformation_matrix(1,3) * Transformation_matrix(0,0);
	inverseT(2,0) = 0;
	inverseT(2,1) = 0;
	inverseT(2,2) = 1;
	inverseT(2,3) = 0;
	inverseT(3,0) = 0;
	inverseT(3,1) = 0;
	inverseT(3,2) = 0;
	inverseT(3,3) = 1;

	return inverseT;

}

Keyframe::matrix4f Keyframe::get_g2o_transformation_matrix(std::vector<Position> Vertex_g2o)
{
	//local x, y, heading for calculating g2o transformation matrix
	float x0, y0, heading0;
	float x1, y1, heading1;
	double theta, diff_x, diff_y;

	matrix4f transformation_matrix_g2o;

	x0 = Vertex_g2o[0].x;
	y0 = Vertex_g2o[0].y;
	heading0 = Vertex_g2o[0].heading;

	x1 = Vertex_g2o[Vertex_g2o.size() - 1].x;
	y1 = Vertex_g2o[Vertex_g2o.size() - 1].y;
	heading1 = Vertex_g2o[Vertex_g2o.size() - 1].heading;

	Coordinate_Normalize(x0, y0, heading0, x1, y1, heading1, theta, diff_x, diff_y);
	transformation_matrix_g2o = angle_to_TransformationMatrix(diff_x, diff_y, theta);

	return transformation_matrix_g2o;


}

void Keyframe::Store_Information(const messages::LocalMap& LocalMapMsgIn)
{
	//store all information into vector LocalMap_All_s
	LocalMap_All localmap_all; //initialize struct

	// localmap_all.x_mean = LocalMapMsgIn.x_mean + LIDAR_TO_ROBOT_X;	//transfer from lidar to robot center
	// localmap_all.y_mean = LocalMapMsgIn.y_mean;
	// localmap_all.z_mean = LocalMapMsgIn.z_mean;
	// localmap_all.var_z = LocalMapMsgIn.var_z;
	localmap_all.x_filter = LocalMapMsgIn.x_filter;
	localmap_all.y_filter = LocalMapMsgIn.y_filter;
	localmap_all.heading_filter = LocalMapMsgIn.heading_filter;
	localmap_all.turnFlag = LocalMapMsgIn.turnFlag;
	localmap_all.stopFlag = LocalMapMsgIn.stopFlag;
	localmap_all.homing_updatedFlag = LocalMapMsgIn.homing_updated_flag;
	// localmap_all.ground_adjacent = (bool) LocalMapMsgIn.ground_adjacent;

	LocalMap_All_s.push_back(localmap_all);

	//store all ground adjacent central points in each localmap into vector LocalMap_ICP_s

	PointCloud ICP_cloudMsgIn; //save ground adjance central points
	ICP_cloudMsgIn.clear();

	PointCloud information_cloudMsgIn;	//save all central points
	information_cloudMsgIn.clear();

	LocalMap_ICP localmap_icp;

	LocalMap_Information localmap_information;

	for(int i = 0; i < LocalMapMsgIn.x_mean.size(); i++)
	{	
		//remove all points inside 5m round
		if(outside_area(LocalMapMsgIn.x_mean[i] + LIDAR_TO_ROBOT_X, LocalMapMsgIn.y_mean[i]))
		{
			//if the cell is near to the ground, save that point for icp pointcloud
			if (LocalMapMsgIn.ground_adjacent[i])
			{
				ICP_cloudMsgIn.push_back (pcl::PointXYZ(LocalMapMsgIn.x_mean[i] + LIDAR_TO_ROBOT_X, LocalMapMsgIn.y_mean[i], 0));
				localmap_icp.z_mean.push_back(LocalMapMsgIn.z_mean[i]);
				localmap_icp.var_z.push_back(LocalMapMsgIn.var_z[i]);
			}

			information_cloudMsgIn.push_back(pcl::PointXYZ(LocalMapMsgIn.x_mean[i] + LIDAR_TO_ROBOT_X, LocalMapMsgIn.y_mean[i], 0)); // save all central points
			localmap_information.z_mean.push_back(LocalMapMsgIn.z_mean[i]);
			localmap_information.var_z.push_back(LocalMapMsgIn.var_z[i]);
			localmap_information.ground_adjacent.push_back(LocalMapMsgIn.ground_adjacent[i]);
		}
	}
	localmap_icp.ICP_cloudMsgIn = ICP_cloudMsgIn;
	LocalMap_ICP_s.push_back(localmap_icp);

	//store all cell information into vector LocalMap_Information_s
	localmap_information.Information_cloudMsgIn = information_cloudMsgIn;
	LocalMap_Information_s.push_back(localmap_information);
}

void Keyframe::getSubVertexEdges()
{
	//*******************************************get sub position and edge for sub g2o************************//
	//sub g2o only work for straight line, not fot turning
	//get edges from current frame to previous frame
	Transformation_Matrix edges_sub;
	matrix4f TframeTomap;
	Position position_sub;

	if(read_index == 1 )
	{
		if(Vertex_temp.size() == 0)
			Vertex_sub.push_back(Vertex[Vertex.size() - 1]);
		else
			Vertex_sub.push_back(Vertex_temp[Vertex_temp.size() - 1]);	//add the keyframe as the first frame for sub g2o
	}
	if(read_index > 1) //start from third frame
	{
		ICP_Result icp_result_sub;
		icp_result_sub = ICP_compute(read_index - 1, read_index, 0, 0);

		
		edges_sub.transformation_matrix = icp_result_sub.transformation_matrix;
		edges_sub.from_index = icp_result_sub.from_index;
		edges_sub.to_index = icp_result_sub.to_index;
		edges_sub.information_matrix = icp_result_sub.information_matrix;

		Edges_sub.push_back(edges_sub);	//edge from current frame to previous frame
	
	}

	Edges_sub.push_back(TreadToprekey_s[TreadToprekey_s.size() - 1]);	//edge from current frame to previous key frame

	
	TframeTomap = TkeyTomap * TreadToprekey_s[TreadToprekey_s.size() - 1].transformation_matrix;

	
	position_sub.x = TframeTomap(0,3);
	position_sub.y = TframeTomap(1,3);
	position_sub.heading = TransformationMatrix_to_angle(TframeTomap);	//radian

	//store current position into vertex sub
	Vertex_sub.push_back(position_sub);
}

void Keyframe::DetectNearestKeyframe()
{
	Keyframe_Information keyframe_pointcloud;
	Position position;
	Transformation_Matrix edges;

	LocalMap_Information localmap_information;

	pcl::PointXYZ searchPoint;	//set the last vertex be the search point
	searchPoint.x = Vertex_temp[Vertex_temp.size() - 2].x;
	searchPoint.y = Vertex_temp[Vertex_temp.size() - 2].y;
	searchPoint.z = 0.0;

	pcl::PointCloud<pcl::PointXYZ>::Ptr Vertex_pointcloud_Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	*Vertex_pointcloud_Ptr = Vertex_pointcloud;	//set the input cloud include the search point and the previous keyframe

	kdtree.setInputCloud(Vertex_pointcloud_Ptr);
	std::vector<int> pointIdxNKNSearch(number_neighbor);
	std::vector<float> pointNKNSquaredDistance(number_neighbor);
	pointIdxNKNSearch.clear();
	pointNKNSquaredDistance.clear();

	bool discard_temp_keyframe = false;

	if(kdtree.nearestKSearch (searchPoint, number_neighbor, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)	//find nearest neighbor
	{	
				

		// segmentation fault problem is here
		for(int i = 0; i < pointIdxNKNSearch.size(); i++)
		{
			ROS_INFO_STREAM(sqrt(pointNKNSquaredDistance[i]));
					
			if(sqrt(pointNKNSquaredDistance[i]) < threshold_permanent_keyframe_distance)
			{
				discard_temp_keyframe = true;
			}
		}
	}

	if(discard_temp_keyframe)
	{

		//clear temp data
		position = Vertex_temp[Vertex_temp.size() - 1];
		edges.transformation_matrix = Edges_temp[Edges_temp.size() -2].transformation_matrix * Edges_temp[Edges_temp.size() -1].transformation_matrix;	//add transformation matrxi together
		edges.from_index = Vertex_temp.size() - 1;
		edges.to_index = Vertex_temp.size() - 2;
		edges.information_matrix = Edges_temp[Edges_temp.size() -1].information_matrix;
		keyframe_pointcloud = KeyframeMap_temp_s[KeyframeMap_temp_s.size() - 1];
		localmap_information = LocalMap_Information_temp_s[LocalMap_Information_temp_s.size() - 1];

		Vertex_temp.clear();
		Edges_temp.clear();
		Vertex_pointcloud_temp.clear();
		KeyframeMap_temp_s.clear();
		LocalMap_Information_temp_s.clear();

		//add the last temp keyframe to temp vector
		Vertex_temp.push_back(position);
		Edges_temp.push_back(edges);
		Vertex_pointcloud_temp.push_back(pcl::PointXYZ(Vertex_temp[Vertex_temp.size() - 1].x, Vertex_temp[Vertex_temp.size() - 1].y, 0)); 
		KeyframeMap_temp_s.push_back(keyframe_pointcloud);
		LocalMap_Information_temp_s.push_back(localmap_information);

	}
	else //add the temp keyframe to permanent keyframe
	{ 
		Vertex.push_back(Vertex_temp[Vertex_temp.size() - 2]);

		//store keyframe position into vertex in pointcloudXYZ datatype
		Vertex_pointcloud.push_back(Vertex_pointcloud_temp[Vertex_pointcloud_temp.size() - 2]);

		edges.transformation_matrix = Edges_temp[Edges_temp.size() - 2].transformation_matrix;
		edges.from_index = Vertex.size() - 1;
		edges.to_index = Vertex.size() - 2;
		edges.information_matrix = Edges_temp[Edges_temp.size() - 2].information_matrix;
		//store transformation between current permanent keyframe and permanent previous keyframe into edges
		Edges.push_back(edges);

		//store keyframe point cloud into keyframe maps
		KeyframeMap_s.push_back(KeyframeMap_temp_s[KeyframeMap_temp_s.size() - 2]);

		//update new permanent keyframe to global map
		matrix4f TpermanentkeyTomap;
		TpermanentkeyTomap = angle_to_TransformationMatrix(Vertex[Vertex.size() - 1].x, Vertex[Vertex.size() - 1].y, Vertex[Vertex.size() - 1].heading);
		int update_Rate = 0;
		update_Rate = Update_GlobalMap(LocalMap_Information_temp_s[LocalMap_Information_temp_s.size() - 2], TpermanentkeyTomap);	//segmatation fault (core dumped) problem is here, size too big

		keyframelist_msg.keyframeList.push_back(Pcak_Keyframe_message(KeyframeMap_s[KeyframeMap_s.size() - 1]));
		keyframePub.publish(keyframelist_msg);


		//clear temp data
		position = Vertex_temp[Vertex_temp.size() - 1];
		edges = Edges_temp[Edges_temp.size() -1];
		keyframe_pointcloud = KeyframeMap_temp_s[KeyframeMap_temp_s.size() - 1];
		localmap_information = LocalMap_Information_temp_s[LocalMap_Information_temp_s.size() - 1];

		Vertex_temp.clear();
		Edges_temp.clear();
		Vertex_pointcloud_temp.clear();
		KeyframeMap_temp_s.clear();
		LocalMap_Information_temp_s.clear();
		//add the last temp keyframe to temp vector
		Vertex_temp.push_back(position);
		Edges_temp.push_back(edges);
		Vertex_pointcloud_temp.push_back(pcl::PointXYZ(Vertex_temp[Vertex_temp.size() - 1].x, Vertex_temp[Vertex_temp.size() - 1].y, 0)); 
		KeyframeMap_temp_s.push_back(keyframe_pointcloud);
		LocalMap_Information_temp_s.push_back(localmap_information);
	}
}

//option 0: regular option 1: homing updated
void Keyframe::TkeyTomap_Pub(int option)
{
	float x_filter_pub = 0;
	float y_filter_pub = 0;
	float heading_filter_pub = 0;

	//publish the IMU data to localization node
	x_filter_pub = LocalMap_All_s[key_index].x_filter;
	y_filter_pub = LocalMap_All_s[key_index].y_filter;
	heading_filter_pub = LocalMap_All_s[key_index].heading_filter;

	//publish TkeyTomap message
	slam::TkeyTomap_msg tkeytomap_msg; 
		
	tkeytomap_msg.x = TkeyTomap(0,3);
	tkeytomap_msg.y = TkeyTomap(1,3);
	tkeytomap_msg.heading = TransformationMatrix_to_angle(TkeyTomap);	//radian

	tkeytomap_msg.x_filter = x_filter_pub;
	tkeytomap_msg.y_filter = y_filter_pub;
	tkeytomap_msg.heading_filter = heading_filter_pub;

	if(option == 1)
 	{
 		tkeytomap_msg.homing_updated = true;
 	}
 	else
 	{
 		tkeytomap_msg.homing_updated = false;
 	}

	TkeyTomapPub.publish(tkeytomap_msg);


}

//option, 0: homing update information store; 1: Edges_sub update; 2: TreadToprekey_s update
void Keyframe::Store_temp_vertex_edges(int option)
{
	Keyframe_Information keyframe_pointcloud;
	Position position;
	Transformation_Matrix edges;

	position.x = TkeyTomap(0,3);
	position.y = TkeyTomap(1,3);
	position.heading = TransformationMatrix_to_angle(TkeyTomap);	//radian

	

	//store keyframe position into vertex_temp 
	Vertex_temp.push_back(position);

	//store keyframe position into vertex in pointcloudXYZ datatype
	Vertex_pointcloud_temp.push_back(pcl::PointXYZ(position.x, position.y, 0.0));

	//get point cloud of the keyframe, save icp pointcloud and verification pointcloud
	keyframe_pointcloud.keyframe_icp_cloud = LocalMap_ICP_s[key_index].ICP_cloudMsgIn;
	keyframe_pointcloud.keyframe_cloud = LocalMap_Information_s[key_index].Information_cloudMsgIn;
	keyframe_pointcloud.x = position.x;
	keyframe_pointcloud.y = position.y;
	keyframe_pointcloud.heading = position.heading;
	keyframe_pointcloud.z_mean = LocalMap_Information_s[key_index].z_mean;
	keyframe_pointcloud.var_z = LocalMap_Information_s[key_index].var_z;



	//store keyframe point cloud into keyframe maps
	KeyframeMap_temp_s.push_back(keyframe_pointcloud);

	if(option == 0)
	{
		edges.transformation_matrix = Eigen::Matrix<float, 4, 4>::Identity();
	}
	else if(option == 1)
	{
		edges.transformation_matrix = Edges_sub[Edges_sub.size() - 1].transformation_matrix;
		edges.information_matrix = Edges_sub[Edges_sub.size() - 1].information_matrix;
	}
	else if(option == 2)
	{
		edges.transformation_matrix = TreadToprekey_s[TreadToprekey_s.size() - 1].transformation_matrix;
		edges.information_matrix = TreadToprekey_s[TreadToprekey_s.size() - 1].information_matrix;
	}
	
	edges.from_index = Vertex_temp.size() - 1;
	edges.to_index = Vertex_temp.size() - 2;
	//store transformation between current keyframe and previous keyframe into edges temp
	Edges_temp.push_back(edges);

	//store temp Information to update global map
	LocalMap_Information_temp_s.push_back(LocalMap_Information_s[key_index]);
}

void Keyframe::Vector_Clean()
{
	LocalMap_All localmap_all;
	LocalMap_ICP localmap_icp;
	LocalMap_Information localmap_information;
	//after generating a new keyframe, clean all map data for free storage
	//keep all information about the keyframe
	localmap_all = LocalMap_All_s[key_index];
	localmap_icp = LocalMap_ICP_s[key_index];
	localmap_information = LocalMap_Information_s[key_index];

	//clean all data
	LocalMap_All_s.clear();
	LocalMap_ICP_s.clear();
	LocalMap_Information_s.clear();
	TreadToprekey_s.clear();
	Vertex_sub.clear();
	Edges_sub.clear();

	//restore the keyframe information as the initial data for next keyframe 
	LocalMap_All_s.push_back(localmap_all);
	LocalMap_ICP_s.push_back(localmap_icp);
	LocalMap_Information_s.push_back(localmap_information);
}

void Keyframe::getg2oResult()
{
	Do_g2o(Vertex_sub, Edges_sub); //do g2o

	ICP_Result icp_result_icp;
	ICP_Result icp_result_g2o;

	matrix4f transformation_matrix_g2o;

	transformation_matrix_g2o = get_g2o_transformation_matrix(Vertex_sub);
	//keep rotation matrix, only change translation
	transformation_matrix_g2o(0,0) = Edges_sub[Edges_sub.size() - 1].transformation_matrix(0,0);
	transformation_matrix_g2o(0,1) = Edges_sub[Edges_sub.size() - 1].transformation_matrix(0,1);
	transformation_matrix_g2o(0,2) = Edges_sub[Edges_sub.size() - 1].transformation_matrix(0,2);
	transformation_matrix_g2o(1,0) = Edges_sub[Edges_sub.size() - 1].transformation_matrix(1,0);
	transformation_matrix_g2o(1,1) = Edges_sub[Edges_sub.size() - 1].transformation_matrix(1,1);
	transformation_matrix_g2o(1,2) = Edges_sub[Edges_sub.size() - 1].transformation_matrix(1,2);
	transformation_matrix_g2o(2,0) = Edges_sub[Edges_sub.size() - 1].transformation_matrix(2,0);
	transformation_matrix_g2o(2,1) = Edges_sub[Edges_sub.size() - 1].transformation_matrix(2,1);
	transformation_matrix_g2o(2,2) = Edges_sub[Edges_sub.size() - 1].transformation_matrix(2,2);
	transformation_matrix_g2o(2,3) = Edges_sub[Edges_sub.size() - 1].transformation_matrix(2,3);
	transformation_matrix_g2o(3,0) = Edges_sub[Edges_sub.size() - 1].transformation_matrix(3,0);
	transformation_matrix_g2o(3,1) = Edges_sub[Edges_sub.size() - 1].transformation_matrix(3,1);
	transformation_matrix_g2o(3,2) = Edges_sub[Edges_sub.size() - 1].transformation_matrix(3,2);
	transformation_matrix_g2o(3,3) = Edges_sub[Edges_sub.size() - 1].transformation_matrix(3,3);

	icp_result_icp = ICP_compute(ref_index, read_index - 1, 0, 0);
	icp_result_g2o = ICP_compute(ref_index, read_index - 1, 0, 3, transformation_matrix_g2o);

	if(icp_result_g2o.verification_result > icp_result_icp.verification_result)
	{
		Edges_sub[Edges_sub.size() - 1].transformation_matrix = transformation_matrix_g2o;
		use_g2o_counter++;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Keyframe_node");

	Keyframe keyframe;

	keyframe.Initialization();

	ros::spin();

	return 0;
}