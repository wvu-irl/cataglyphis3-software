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
#include <pcl/common/transforms.h>	//for transform pointcloud using transformation matrix


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

	//define a struct for ICP result
	typedef struct ICP_Result
	{
		Matrix4f transformation_matrix;
		bool verification_result;
		int from_index;
		int to_index;
		double overlap;
	}ICP_Result;

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
	double ratio_height_varification;

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
	// PointCloud refPointCloud;
	// PointCloud readPointCloud;
	// PointCloud keyPointCloud;


	//globe verify pointcloud
	// PointCloud V_refPointCloud;
	// PointCloud V_readPointCloud;
	// PointCloud V_keyPointCloud;

	//globe matrix
	Matrix4f TreadToref;
	Matrix4f TrefTomap;
	Matrix4f TreadTomap;
	Matrix4f guessT;

	//lists of all maps
	std::vector<LocalMap_All> LocalMap_All_s;
	std::vector<LocalMap_ICP> LocalMap_ICP_s;
	std::vector<LocalMap_Varification> LocalMap_Varification_s;

	//count input index
	int messages_input_index;

	//ICP index
	int ref_index;
	int read_index;
	int key_index;

	void getlocalmapcallback(const messages::LocalMap& LocalMapMsgIn);
	// void set_Navfilter_data(float IMU_x, float IMU_y, float IMU_heading);
	bool ICP_compute(int message_input_index, ICP_Result &icp_result);	//*************change return type to struct include tranformation matrix, from index, to index, overlap, true of false***********//
	bool outside_rectangle(float rectangle_x, float rectangle_y);		//determine if the point is inside 2.24m rectangle
	bool ICP_varification(int ref_v_index, int read_v_index, Matrix4f T, double &overlap);





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

	//parameters for varification
	ratio_height_varification = 0.5; 

	//parameters for points filter
	edge_rectangle_filter = 2.24; //length of edgs of rectangle filter

	//initialize all vectors
	LocalMap_All_s.clear();
	LocalMap_ICP_s.clear();
	LocalMap_Varification_s.clear();

	//index for ICP calculation
	messages_input_index = 0;
	ref_index = 0;
	read_index = 0;
	key_index = 0;

	//transformation matrix
	TreadToref = Eigen::Matrix<float, 4, 4>::Identity();
	TrefTomap = Eigen::Matrix<float, 4, 4>::Identity();
	TreadTomap = Eigen::Matrix<float, 4, 4>::Identity();
	guessT = Eigen::Matrix<float, 4, 4>::Identity();


}



void Keyframe::getlocalmapcallback(const messages::LocalMap& LocalMapMsgIn)
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
		//localmap_all.ground_adjacent = LocalMapMsgIn.ground_adjacent;

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
					ICP_cloudMsgIn.push_back (pcl::PointXYZ(LocalMapMsgIn.x_mean[i], LocalMapMsgIn.y_mean[i], 0));
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

		LocalMap_Varification_s.push_back(localmap_varification);
	
		//when a new local map message come, messages_input_index add one
		messages_input_index++;


	// 	//do ICP to get transformation matrix from read to reference
		ICP_Result icp_result;
		ICP_compute(messages_input_index, icp_result);

	// 	//verify the transformation matrix from ICP using hight information

	}
	
}

//**************************ICP calculation**********************//
bool Keyframe::ICP_compute(int message_input_index, ICP_Result &icp_result)
{
	double overlap;
	bool verification_result;
	//ICP_Result icp_result;

	PointCloudPtr refPointCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr readPointCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
	//if keyframe update, reset the reference frame as the new keyframe
	if(ref_index != key_index)
	{
		ref_index = key_index;

		// TrefTomap = TreadTomap;
	}

	read_index = message_input_index;

	x0 = LocalMap_All_s[ref_index].x_filter;
	y0 = LocalMap_All_s[ref_index].y_filter;
	heading0 = LocalMap_All_s[ref_index].heading_filter;

	x1 = LocalMap_All_s[read_index].x_filter;
	y1 = LocalMap_All_s[read_index].y_filter;
	heading1 = LocalMap_All_s[read_index].heading_filter;

	//using Nav filter data to calculate guess transformation matrix
	theta = heading1 - heading0;
	theta = theta * PI /180; // degree to radian
	diff_x = x1 - x0;
	diff_y = y1 - y0;

	sin_theta = sin(theta);
	cos_theta = cos(theta);

	guessT(0,0) = (float)cos_theta;  guessT(0,1) = (float)(-sin_theta);  guessT(0,2) = 0;  guessT(0,3) = (float)diff_x;
	guessT(1,0) = (float)sin_theta;  guessT(1,1) = (float)cos_theta;     guessT(1,2) = 0;  guessT(1,3) = (float)diff_y;
	guessT(2,0) = 0;                 guessT(2,1) = 0;					 guessT(2,2) = 1;  guessT(2,3) = 0;
	guessT(3,0) = 0;                 guessT(3,1) = 0;                    guessT(3,2) = 0;  guessT(3,3) = 1;

	//icp calculation
	*refPointCloudPtr = LocalMap_ICP_s[ref_index].ICP_cloudMsgIn;
	*readPointCloudPtr = LocalMap_ICP_s[read_index].ICP_cloudMsgIn;
	PointCloud Final;

	icp.setInputSource(readPointCloudPtr);
	icp.setInputTarget(refPointCloudPtr);
	icp.align(Final, guessT);

	Matrix4f FinalTransformation;
	FinalTransformation = icp.getFinalTransformation();

	// return icp.getFinalTransformation(); //resutl of ICP
	verification_result = ICP_varification(ref_index, read_index, FinalTransformation, overlap);

	icp_result.transformation_matrix = icp.getFinalTransformation();
	icp_result.verification_result = verification_result;
	icp_result.from_index = read_index;
	icp_result.to_index = ref_index;
	icp_result.overlap = overlap;

	//dynamically allocated pointers have to be deleted
	// delete refPointCloudPtr;
	// delete readPointCloudPtr;

	return true;
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

//********************varification**********************//
bool Keyframe::ICP_varification(int ref_v_index, int read_v_index, Matrix4f T, double &overlap)
{
	//define pointer to get the points information
	PointCloudPtr read_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  	PointCloudPtr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	PointCloudPtr ref_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  	*read_cloud = LocalMap_Varification_s[read_v_index].Varification_cloudMsgIn;
  	*ref_cloud = LocalMap_Varification_s[ref_v_index].Varification_cloudMsgIn;

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
  		read_z_value = LocalMap_Varification_s[read_v_index].z_mean[all_correspondences[i].index_query];
  		ref_z_value = LocalMap_Varification_s[ref_v_index].z_mean[all_correspondences[i].index_match];
  		ref_var_z = LocalMap_Varification_s[ref_v_index].var_z[all_correspondences[i].index_match];
  		if(read_z_value < ref_z_value + 3 * ref_var_z && read_z_value > ref_z_value - 3 * ref_var_z)
  			count++;
  	}

  	//dynamically allocated pointers have to be deleted
  	// delete read_cloud;
  	// delete transformed_cloud;
  	// delete ref_cloud;

  	if(count / (double) all_correspondences.size() > ratio_height_varification)
  		return true;
  	else
  		return false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Keyframe_node");

	Keyframe keyframe;

	keyframe.Initialization();

	ros::spin();

	return 0;
}