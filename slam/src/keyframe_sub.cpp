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

class Keyframe_sub
{
public:
	Keyframe_sub();

	int count = 0;

protected:

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

	typedef struct Localmap_whole_withIMU
	{
		std::vector<float> x_mean;
		std::vector<float> y_mean;
		std::vector<float> z_mean;
		std::vector<float> var_z;
		std::vector<bool> ground_adjacent;

		float x_filter;
		float y_filter;
		float heading_filter;
	}Localmap_whole_withIMU;

	
	typedef struct Localmap_ICP_calculation
	{
		PointCloud ICP_cloudMsgIn;

	}Localmap_ICP_calculation;

	typedef struct Localmap_varification
	{
		PointCloud Varification_cloudMsgIn;
		std::vector<float> z_mean;
		std::vector<float> var_z;
	
	}Localmap_varification;


	ros::NodeHandle node;

	//subscribe
	ros::Subscriber localmapSub;

	std::vector<Localmap_whole_withIMU> LocalmapWithIMU_s;
	std::vector<Localmap_ICP_calculation> Localmap_centralpoints_s; 
	std::vector<Localmap_varification> Localmap_withheight_s;



	void getlocalmapcallback(const messages::LocalMap& LocalMapMsgIn);

};

Keyframe_sub::Keyframe_sub()
{
	localmapSub = node.subscribe("/lidar/lidarfilteringnode/localmap", 1, &Keyframe_sub::getlocalmapcallback, this);
};

void Keyframe_sub::getlocalmapcallback(const messages::LocalMap& LocalMapMsgIn)
{
	if(LocalMapMsgIn.new_data)
	{

		PointCloud ICP_cloudMsgIn;
		ICP_cloudMsgIn.clear();
		PointCloud Varification_cloudMsgIn;
		Varification_cloudMsgIn.clear();

		Localmap_ICP_calculation localmap_icp_calculation;
		Localmap_varification localmap_varification;
		//***********************************************************************//
		// store all information into localmap with IMU vector
		Localmap_whole_withIMU Localmap_whole_withIMU;

		Localmap_whole_withIMU.x_mean = LocalMapMsgIn.x_mean;
		Localmap_whole_withIMU.y_mean = LocalMapMsgIn.y_mean;
		Localmap_whole_withIMU.z_mean = LocalMapMsgIn.z_mean;
		Localmap_whole_withIMU.var_z = LocalMapMsgIn.var_z;

		Localmap_whole_withIMU.x_filter = LocalMapMsgIn.x_filter;
		Localmap_whole_withIMU.y_filter = LocalMapMsgIn.y_filter;
		Localmap_whole_withIMU.heading_filter = LocalMapMsgIn.heading_filter;

		LocalmapWithIMU_s.push_back(Localmap_whole_withIMU);

		// ROS_INFO_STREAM(LocalmapWithIMU_s[count].x_filter);	

		// count++;
		// ROS_INFO_STREAM(count);

		//***********************************************************************//
		// vector ground adjanced points for ICP calculating


		//input the pointcloud for icp and save localmap
		for(int i = 0; i < LocalMapMsgIn.x_mean.size(); i++)
		{	
				//if the cell is near to the ground, save that point for icp pointcloud
				if (LocalMapMsgIn.ground_adjacent[i])
				{
					ICP_cloudMsgIn.push_back (pcl::PointXYZ(LocalMapMsgIn.x_mean[i], LocalMapMsgIn.y_mean[i], 0));
				}

				Varification_cloudMsgIn.push_back (pcl::PointXYZ(LocalMapMsgIn.x_mean[i], LocalMapMsgIn.y_mean[i], 0));

		}
		localmap_icp_calculation.ICP_cloudMsgIn = ICP_cloudMsgIn;

		Localmap_centralpoints_s.push_back(localmap_icp_calculation);

		// ROS_INFO_STREAM(Localmap_centralpoints_s[count].ICP_cloudMsgIn.size());	

		// count++;
		// ROS_INFO_STREAM(count);

		//***********************************************************************//
		// vector all cells information include height information into local maps
		localmap_varification.Varification_cloudMsgIn = Varification_cloudMsgIn;
		localmap_varification.z_mean = LocalMapMsgIn.z_mean;
		localmap_varification.var_z = LocalMapMsgIn.var_z;

		Localmap_withheight_s.push_back(localmap_varification);

		ROS_INFO_STREAM(Localmap_withheight_s[count].z_mean[10]);	

		count++;
		ROS_INFO_STREAM(count);
	}



};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Keyframe_sub_node");

	Keyframe_sub keyframe_sub;

	ros::spin();

	return 0;
}