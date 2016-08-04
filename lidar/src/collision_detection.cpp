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
	//_registration_new = false;
	_sub_velodyne = _nh.subscribe("/velodyne_points", 1, &CollisionDetection::registrationCallback, this);

	//predictive avoidance service
	returnHazardMapServ = _nh.advertiseService("/lidar/collisiondetection/createroihazardmap", &CollisionDetection::returnHazardMap, this);
	
	//collision output
	_collision_status = 0;
}

// void CollisionDetection::service(messages::::Request &req, messages::::Response &res)
// {
// 	int index;
// 	index = req.index;

// 	res.x = x;
// 	res.y = y;
// }


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
		//_registration_new = true;
		return true;
	}
	else
	{
		//_registration_new = false;
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
				if(fabs(atan2( (_LIDAR_HEIGHT - cloud->points[i].z),cloud->points[i].x )) > _SAFE_ENVELOPE_ANGLE )
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
		_collision_status = 0;
		ROS_INFO("No Collision...");
		return 0;
	}
}

int CollisionDetection::doMathRANSAC() // SECOND LAYER: RANSAC FIT A PLANE
{

}

bool CollisionDetection::doPredictiveAovidance()
{
	//reference point cloud for processing
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	*cloud = _input_cloud;

	int preditive_region_length = 7;
	int preditive_region_wide = 5; //this is on each side, so the total is 5*2 =10

	//remove points based on hard thresholds (too far, too high, too low)
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(0,preditive_region_length);
	pass.filter(*cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-preditive_region_wide,preditive_region_wide);
	pass.filter(*cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-5,5); //positive z is down, negative z is up
	pass.filter(*cloud);

	//create segmentation object for fitting a plane to points in the full cloud using RANSAC (assuming the fit plane represents the ground)
	pcl::SACSegmentation<pcl::PointXYZI> seg_plane;
	seg_plane.setOptimizeCoefficients (true); //optional (why is this optional??)
	seg_plane.setModelType (pcl::SACMODEL_PLANE);
	seg_plane.setMethodType (pcl::SAC_RANSAC);
	seg_plane.setMaxIterations (1000); //max iterations for RANSAC
	seg_plane.setDistanceThreshold (0.75); //ground detection threshold parameter
	seg_plane.setInputCloud (cloud); //was raw_cloud

	//segment the points fitted to the plane using ransac
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ()); //what is this? (coefficients for fitted plane?)
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ()); //what is this? (inliers for points that fit the plane?)
	seg_plane.segment (*inliers, *coefficients);

	//seperate the ground points and the points above the ground (object points)
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);

	extract.setNegative (false);
	pcl::PointCloud<pcl::PointXYZI>::Ptr ground_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	extract.filter (*ground_filtered);

	extract.setNegative (true);
	pcl::PointCloud<pcl::PointXYZI>::Ptr object_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	extract.filter (*object_filtered);

	//project points on the xy plane, and that is the local hazard map
	pcl::PointCloud<pcl::PointXYZI>::Ptr object_filtered_projection (new pcl::PointCloud<pcl::PointXYZI>);
	*object_filtered_projection = *object_filtered;
	for (int i=0; i<object_filtered->points.size(); i++)
	{
		object_filtered_projection->points[i].z=0;
	}


	int bin_num = 10;
	int bin_counter[bin_num];
	for (int i=0; i<bin_num; i++)
	{
		bin_counter[i] = 0;
	}

	cout << object_filtered_projection->points.size() << endl;
	for(int i=0; i<object_filtered_projection->points.size(); i++)//every point in the predictive region
	{
		bin_counter[(int)ceil((floor(object_filtered_projection->points[i].y) + preditive_region_wide)/(preditive_region_wide*2/bin_num))] += 1; 
	}

	int bin_checker = 0;
	for(int i=0; i<bin_num; i++)
	{
		if(bin_counter[i] == 0)
		{
			bin_checker += 1;
		}
	}

	cout << "Value of bin_checker is " << bin_checker << endl;
	float threshold_ratio = 0.7;
	if(bin_checker <= (int)(bin_num*threshold_ratio)) //less than 70% of the bins are clear
	{
		//return true;//high risk area in front
		cout << "High risk area in front" << endl;

		//trigger the hazard map generation function
		pcl::PointCloud<pcl::PointXYZI>::Ptr hazard_cloud (new pcl::PointCloud<pcl::PointXYZI>);
	    *hazard_cloud = _input_cloud;

	    int hazard_map_size = 30; //so each side is 30*2 = 60 

		//remove points based on hard thresholds (too far, too high, too low)
		//pcl::PassThrough<pcl::PointXYZI> pass;
		pass.setInputCloud(hazard_cloud);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(-hazard_map_size,hazard_map_size);
		pass.filter(*hazard_cloud);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(-hazard_map_size,hazard_map_size);
		pass.filter(*hazard_cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(-5,5); //positive z is down, negative z is up
		pass.filter(*hazard_cloud);

		//create segmentation object for fitting a plane to points in the full cloud using RANSAC (assuming the fit plane represents the ground)
		pcl::SACSegmentation<pcl::PointXYZI> plane;
		plane.setOptimizeCoefficients (true); //optional (why is this optional??)
		plane.setModelType (pcl::SACMODEL_PLANE);
		plane.setMethodType (pcl::SAC_RANSAC);
		plane.setMaxIterations (1000); //max iterations for RANSAC
		plane.setDistanceThreshold (0.75); //ground detection threshold parameter
		plane.setInputCloud (hazard_cloud); //was raw_cloud

		//segment the points fitted to the plane using ransac
		pcl::ModelCoefficients::Ptr coefficients_hazard (new pcl::ModelCoefficients ()); //what is this? (coefficients for fitted plane?)
		pcl::PointIndices::Ptr inliers_hazard (new pcl::PointIndices ()); //what is this? (inliers for points that fit the plane?)
		seg_plane.segment (*inliers_hazard, *coefficients_hazard);

		//seperate the ground points and the points above the ground (object points)
		//pcl::ExtractIndices<pcl::PointXYZI> extract;
		extract.setInputCloud (hazard_cloud);
		extract.setIndices (inliers_hazard);

		extract.setNegative (true);
		pcl::PointCloud<pcl::PointXYZI>::Ptr hazard_filtered (new pcl::PointCloud<pcl::PointXYZI>);
		extract.filter (*hazard_filtered);

		//project points on the xy plane, and that is the local hazard map
		pcl::PointCloud<pcl::PointXYZI>::Ptr hazard_filtered_projection (new pcl::PointCloud<pcl::PointXYZI>);
		*hazard_filtered_projection = *hazard_filtered;
		for (int i=0; i<hazard_filtered->points.size(); i++)
		{
			hazard_filtered_projection->points[i].z=0;
		}

		//voxel grid filter
		pcl::VoxelGrid<pcl::PointXYZI> sor;
		sor.setInputCloud (hazard_filtered_projection);
		sor.setLeafSize (1.0f, 1.0f, 1.0f);
		sor.filter (*hazard_filtered_projection);

		_hazard_x.clear();
		_hazard_y.clear();
		for(int i = 0; i< hazard_filtered_projection->points.size(); i++)
		{
			_hazard_x.push_back(hazard_filtered_projection->points[i].x);
			_hazard_y.push_back(hazard_filtered_projection->points[i].y);
		}

		//the above hazard_filter_projection is the hazard map and we need to publish this map
		cout << "Hazard map get published !!!" << endl;
		return true;
	}
	else
	{
		cout << "No risk in front" << endl;
		return false;
		//relative safe region
	}
	bin_checker = 0; 
}

bool CollisionDetection::returnHazardMap(messages::CreateROIHazardMap::Request &req, messages::CreateROIHazardMap::Response &res)
{
	res.x_mean.clear();
	res.y_mean.clear();
	for(int i=0; i<_hazard_x.size();i++)
	{
		res.x_mean.push_back(_hazard_x[i]);
		res.y_mean.push_back(_hazard_y[i]);
	}
	return true;
}