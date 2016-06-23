#include <lidar/collision_detection.hpp>

CollisionDetection::CollisionDetection()
{
	//rotation from lidar to robot body frame (rotation)
	_R_lidar_to_robot(0,0) = 1;
	_R_lidar_to_robot(0,1) = 0;
	_R_lidar_to_robot(0,2) = 0;
	_R_lidar_to_robot(1,0) = 0;
	_R_lidar_to_robot(1,1) = -1;
	_R_lidar_to_robot(1,2) = 0;
	_R_lidar_to_robot(2,0) = 0;
	_R_lidar_to_robot(2,1) = 0;
	_R_lidar_to_robot(2,2) = -1;

	//velodyne callback initializations
	_registration_counter = 0;
	_registration_counter_prev = 0;
	_registration_new = false;
	_sub_velodyne = _nh.subscribe("/velodyne_points", 1, &CollisionDetection::registrationCallback, this);

	//collision output
	_collision_status = 0;
}

void CollisionDetection::registrationCallback(pcl::PointCloud<pcl::PointXYZI> const &input_cloud)
{
	pcl::PointCloud<pcl::PointXYZI> temp_cloud = input_cloud;
    _registration_counter = _registration_counter + 1;

    //create 4x4 transformation with 0 translation
    Eigen::Matrix4f T_temporary;
    T_temporary(0,0) = _R_lidar_to_robot(0,0);
    T_temporary(0,1) = _R_lidar_to_robot(0,1);
    T_temporary(0,2) = _R_lidar_to_robot(0,2);
    T_temporary(0,3) = 0;

    T_temporary(1,0) = _R_lidar_to_robot(1,0);
    T_temporary(1,1) = _R_lidar_to_robot(1,1);
    T_temporary(1,2) = _R_lidar_to_robot(1,2);
    T_temporary(1,3) = 0;

    T_temporary(2,0) = _R_lidar_to_robot(2,0);
    T_temporary(2,1) = _R_lidar_to_robot(2,1);
    T_temporary(2,2) = _R_lidar_to_robot(2,2);
    T_temporary(2,3) = 0;

    T_temporary(3,0) = 0;
    T_temporary(3,1) = 0;
    T_temporary(3,2) = 0;
    T_temporary(3,3) = 1;

    //apply rotation to temp_cloud (note translation is 0)
    pcl::transformPointCloud(temp_cloud, _input_cloud, T_temporary);
}

void CollisionDetection::setPreviousCounters()
{
	_registration_counter_prev = _registration_counter;
}

bool CollisionDetection::newPointCloudAvailable()
{
	if(_registration_counter != _registration_counter_prev)
	{
		_registration_new = true;
		return true;
	}
	else
	{
		_registration_new = false;
		return false;
	}
}

void CollisionDetection::packCollisionMessage(messages::CollisionOut &msg)
{	
	msg.collision = _collision_status;
	msg.distance_to_collision = 4.0;
}

int CollisionDetection::doMathSafeEnvelope() // FIRST LAYER: SAFE ENVELOPE
{
	//reference point cloud for processing
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	*cloud = _input_cloud;

	//check for collision points
	int collision_point_counter = 0;
	int collision_left_counter = 0;
	int collision_right_counter = 0;
	
	for(int i=0; i<cloud->points.size(); i++)
	{
		//check if point in corridor (width check)
		if(cloud->points[i].y < 0.5*_CORRIDOR_WIDTH && cloud->points[i].y > -0.5*_CORRIDOR_WIDTH)
		{
			//check if point in corridor (length check)
			if(cloud->points[i].x > 0 && cloud->points[i].x < _CORRIDOR_LENGTH)
			{
				//check if point is outside of safe envelope
				if(fabs(atan2( (_LIDAR_HEIGHT + cloud->points[i].z),cloud->points[i].x )) > _SAFE_ENVELOPE_ANGLE )
				{
					//increment collision counter
					collision_point_counter++;
					if(cloud->points[i].y>0)
					{
						collision_right_counter++; //right point counter
					}
					else
					{
						collision_left_counter++; //left point counter
					}
				}
			}
		}
	}

	//check if points exceed threshold
	if(collision_point_counter > _TRIGGER_POINT_THRESHOLD)
	{
		//determine side of collision
		if(collision_left_counter > collision_right_counter)
		{
			_collision_status = 1;
			ROS_INFO("COLLISION ON LEFT");
			return 1;
		}
		else
		{
			_collision_status = 2;
			ROS_INFO("COLLISION ON RIGHT");		
			return 2;		
		}
	}
	else
	{
		_collision_status = 3;
		ROS_INFO("No Collision...");
		return 0;
	}
}

int CollisionDetection::doMathRANSAC() // SECOND LAYER: RANSAC FIT A PLANE
{

}