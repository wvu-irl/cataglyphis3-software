#include <lidar/lidar_filtering.hpp>

LidarFilter::LidarFilter()
{
	//navigation filter callback initialization
	_navigation_filter_x = 0;
	_navigation_filter_y = 0;
	_navigation_filter_roll = 0;
	_navigation_filter_pitch = 0;
	_navigation_filter_heading = 0;
	_navigation_filter_counter = 0;
	_navigation_filter_counter_prev = 0;
	_sub_navigation = _nh.subscribe("navigation/navigationfilterout/navigationfilterout", 1, &LidarFilter::navigationFilterCallback, this);

	//ExecInfo callback initialization
	_execinfo_turnflag = false;
	_sub_execinfo = _nh.subscribe("control/exec/info", 1, &LidarFilter::rexrcinforCallback, this);

	//rotation from robot to homing beacon (pitch and roll rotation only)
	_R_tilt_robot_to_beacon = Eigen::Matrix3f::Identity();

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
	_sub_velodyne = _nh.subscribe("/velodyne_points", 1, &LidarFilter::registrationCallback, this);

	//homing initialization
	_homing_x = 0;
	_homing_y = 0;
	_homing_heading = 0;
	_homing_found = false;

	//clouds stitch
	pcl::PointCloud<pcl::PointXYZI> piece_one;
	pcl::PointCloud<pcl::PointXYZI> piece_two;
}

void LidarFilter::navigationFilterCallback(const messages::NavFilterOut::ConstPtr &msg)
{
	_navigation_filter_x = msg->x_position; //meters
	_navigation_filter_y = msg->y_position; //meters
	_navigation_filter_roll = msg->roll*3.14159265/180.0; //radians
	_navigation_filter_pitch = msg->pitch*3.14159265/180.0; //radians
	_navigation_filter_heading = msg->heading*3.14159265/180.0; //radians
	_navigation_filter_counter = _navigation_filter_counter + 1;

	//roll rotation using navigation data
	Eigen::Matrix3f _R_roll;
	_R_roll(0,0) = 1;
	_R_roll(0,1) = 0;
	_R_roll(0,2) = 0;
	_R_roll(1,0) = 0;
	_R_roll(1,1) = cos(_navigation_filter_roll);
	_R_roll(1,2) = -sin(_navigation_filter_roll);
	_R_roll(2,0) = 0;
	_R_roll(2,1) = sin(_navigation_filter_roll);
	_R_roll(2,2) = cos(_navigation_filter_roll);

	//set roll transformation to identity for testing
	// _R_roll(0,0) = 1;
	// _R_roll(0,1) = 0;
	// _R_roll(0,2) = 0;
	// _R_roll(1,0) = 0;
	// _R_roll(1,1) = 1;
	// _R_roll(1,2) = 0;
	// _R_roll(2,0) = 0;
	// _R_roll(2,1) = 0;
	// _R_roll(2,2) = 1;

	//pitch rotation using navigation data
	Eigen::Matrix3f _R_pitch;
	_R_pitch(0,0) = cos(_navigation_filter_pitch);
	_R_pitch(0,1) = 0;
	_R_pitch(0,2) = -sin(_navigation_filter_pitch);
	_R_pitch(1,0) = 0;
	_R_pitch(1,1) = 1;
	_R_pitch(1,2) = 0;
	_R_pitch(2,0) = sin(_navigation_filter_pitch);
	_R_pitch(2,1) = 0;
	_R_pitch(2,2) = cos(_navigation_filter_pitch);

	//set pitch transformation to identity for testing
	// _R_pitch(0,0) = 1;
	// _R_pitch(0,1) = 0;
	// _R_pitch(0,2) = 0;
	// _R_pitch(1,0) = 0;
	// _R_pitch(1,1) = 1;
	// _R_pitch(1,2) = 0;
	// _R_pitch(2,0) = 0;
	// _R_pitch(2,1) = 0;
	// _R_pitch(2,2) = 1;

	//rotation from lidar to robot body frame (rotation)
	_R_tilt_robot_to_beacon = _R_roll*_R_pitch;
}

void LidarFilter::rexrcinforCallback(const messages::ExecInfo::ConstPtr &exec_msg)
{
	_execinfo_turnflag = exec_msg->turnFlag;
}

void LidarFilter::registrationCallback(pcl::PointCloud<pcl::PointXYZI> const &input_cloud)
{
	pcl::PointCloud<pcl::PointXYZI> temp_cloud = input_cloud;
    _registration_counter = _registration_counter + 1;

    //calculate rotation from lidar to robot body then compensate for the robot tilt
    Eigen::Matrix3f R_temporary = _R_tilt_robot_to_beacon*_R_lidar_to_robot;

    //create 4x4 transformation with 0 translation
    Eigen::Matrix4f T_temporary;
    T_temporary(0,0) = R_temporary(0,0);
    T_temporary(0,1) = R_temporary(0,1);
    T_temporary(0,2) = R_temporary(0,2);
    T_temporary(0,3) = 0;

    T_temporary(1,0) = R_temporary(1,0);
    T_temporary(1,1) = R_temporary(1,1);
    T_temporary(1,2) = R_temporary(1,2);
    T_temporary(1,3) = 0;

    T_temporary(2,0) = R_temporary(2,0);
    T_temporary(2,1) = R_temporary(2,1);
    T_temporary(2,2) = R_temporary(2,2);
    T_temporary(2,3) = 0;

    T_temporary(3,0) = 0;
    T_temporary(3,1) = 0;
    T_temporary(3,2) = 0;
    T_temporary(3,3) = 1;

    //apply rotation to temp_cloud (note translation is 0)
    pcl::transformPointCloud(temp_cloud, _input_cloud, T_temporary);
}

void LidarFilter::setPreviousCounters()
{
	_navigation_filter_counter_prev = _navigation_filter_counter;
	_registration_counter_prev = _registration_counter;
}

bool LidarFilter::newPointCloudAvailable()
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

void LidarFilter::stitchClouds()
{
	if(_registration_counter % 2 == 1)
	{
		_piece_one = _input_cloud;
	}
	else if(_registration_counter % 2 == 0 && _registration_counter != 0) //stitch two clouds together
	{
		_piece_two = _input_cloud;
		_input_cloud = _piece_one + _piece_two;
	}
}

void LidarFilter::packLocalMapMessage(messages::LocalMap &msg)
{	
	//clear message
	msg.x_mean.clear();
	msg.y_mean.clear();
	msg.z_mean.clear();
	msg.var_z.clear();
	msg.ground_adjacent.clear(); 

	//populate local map
	for(int i=0; i<_local_grid_map.size(); i++)
	{
	    msg.x_mean.push_back(_local_grid_map_new[i][0]);
	    msg.y_mean.push_back(_local_grid_map_new[i][1]);
	    msg.z_mean.push_back(_local_grid_map_new[i][2]);
	    msg.var_z.push_back(_local_grid_map_new[i][3]);
	    //msg.ground_adjacent.push_back(1);
	    msg.ground_adjacent.push_back(_local_grid_map_new[i][4]);
	    //ROS_INFO_STREAM("x: "<<msg.x_mean[i]);
	}

	//forward relavent navigation information
	msg.x_filter = this->_navigation_filter_x;
	msg.y_filter = this->_navigation_filter_y;
	msg.heading_filter = this->_navigation_filter_heading;

	//forward relavent turing flag information
	msg.turnFlag = this->_execinfo_turnflag;

	//flag the data as new
	msg.new_data = _registration_new;

	//ROS_INFO_STREAM("pointcloud size: " << msg.x_mean.size() );
}

void LidarFilter::packHomingMessage(messages::LidarFilterOut &msg)
{
	//populate homing information
	if(_homing_found)
	{
		msg.homing_x = _homing_x;
		msg.homing_y = _homing_y;
		msg.homing_heading = _homing_heading;
		msg.homing_found = _homing_found;		
	}
	else
	{
		msg.homing_x = 0;
		msg.homing_y = 0;
		msg.homing_heading = 0;
		msg.homing_found = _homing_found;			
	}

}

void LidarFilter::doMathMapping()
{
	//copy point cloud to local variable
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	*cloud = _input_cloud;

	ROS_INFO("*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*");
	//ROS_INFO("Running callback function with %i points.",cloud->points.size());

	//test the intensity data
    //for (int jj=1; jj<cloud->points.size();jj++)
    //{
    //	cout << cloud->points[jj].intensity << endl;
    //}

	//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
	//-*-*-*-*-*-*-*-*FILTER POINTS FOR BUILDING LOCAL MAP-*-*-*-*-*-*-*-*-*
	//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

	//define a new point cloud with the size of grid map size (1x1 m grid) for object
	pcl::PointCloud<pcl::PointXYZI> new_point;
	new_point.width  = (2*map_range)*(2*map_range);
	new_point.height = 1;
	new_point.points.resize (new_point.width * new_point.height);

	//define a new point cloud with the size of grid map size (1x1 m grid) for ground points
	pcl::PointCloud<pcl::PointXYZI> ground_point;
	ground_point.width  = (2*map_range)*(2*map_range);
	ground_point.height = 1;
	ground_point.points.resize (ground_point.width * ground_point.height);
	
	//remove points based on hard thresholds (too far, too high, too low)
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-map_range,map_range);
	pass.filter(*cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-map_range,map_range);
	pass.filter(*cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-1*threshold_tree_height,1.5); //positive z is down, negative z is up
	pass.filter(*cloud);

	//save point cloud after hard thresholds
	// std::stringstream ss1;
	// ss1 << "ss1.pcd";
	// pcl::io::savePCDFile( ss1.str(), *cloud);

	//create segmentation object for fitting a plane to points in the full cloud using RANSAC (assuming the fit plane represents the ground)
	pcl::SACSegmentation<pcl::PointXYZI> seg_plane;
	seg_plane.setOptimizeCoefficients (true); //optional (why is this optional??)
	seg_plane.setModelType (pcl::SACMODEL_PLANE);
	seg_plane.setMethodType (pcl::SAC_RANSAC);
	seg_plane.setMaxIterations (1000); //max iterations for RANSAC
	seg_plane.setDistanceThreshold (0.15); //ground detection threshold parameter
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

	//save point cloud after ground removal
	// std::stringstream ss2;
	// ss2 << "ss2.pcd";
	// pcl::io::savePCDFile( ss2.str(), *object_filtered);

	//project points on the xy plane
	pcl::PointCloud<pcl::PointXYZI>::Ptr object_filtered_projection (new pcl::PointCloud<pcl::PointXYZI>);
	*object_filtered_projection = *object_filtered;
	for (int i=0; i<object_filtered->points.size(); i++)
	{
		object_filtered_projection->points[i].z=0;
	}

	//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
	//-*-*-*-*-*-*-*-*-*-*--*-*-BUILD LOCAL MAP*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
	//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

	//define variables used in this section
	std::vector<float> point;
	std::vector<std::vector<std::vector<float> > > grid_map_cells((map_range*2)*(map_range*2));
	std::vector<std::vector<float> > local_grid_map_temp;
	int index = 0;
	_local_grid_map.clear();

	//grid_map_cells is a vector of vectors, each element of it is a grid in the local map which includes 0-N points
	for (int i = 0; i< object_filtered->points.size(); i++)
	{
	    point.clear();
	    point.push_back(object_filtered->points[i].x);
	    point.push_back(object_filtered->points[i].y);
	    point.push_back(object_filtered->points[i].z);

	    //the index checks which gird a point belongs to 
	    index = floor(object_filtered->points[i].x + map_range) + floor(object_filtered->points[i].y + map_range)*(map_range*2);
	    grid_map_cells[index].push_back(point);
	}



	//do the calculation
	for (int i = 0; i < grid_map_cells.size(); i++) // for every cell
	{
		//define variables used to calculate mean x y z and variance of z
		float total_x = 0;
		float total_y = 0;
		float total_z = 0;
		float average_x = 0;
		float average_y = 0;
		float average_z = 0;
		float variance_z = 0;

	    for (int j = 0; j < grid_map_cells[i].size(); j++)
	    {
	        total_x += grid_map_cells[i][j][0];
	        total_y += grid_map_cells[i][j][1];
	        total_z += grid_map_cells[i][j][2];
	    }
	    average_x = total_x/grid_map_cells[i].size();
	    average_y = total_y/grid_map_cells[i].size();
	    average_z = total_z/grid_map_cells[i].size();
	    for (int j = 0; j < grid_map_cells[i].size(); j++)
	    {
	        variance_z = (grid_map_cells[i][j][2]-average_z) * (grid_map_cells[i][j][2]-average_z);
	    }
	    variance_z = sqrt(variance_z);

	    //the point should have at least one of the x, y or z not equal to 0 inorder to be included in the local map
	    if (total_x || total_y || total_z) //this is strange, what is this supposed to do?
	    {
	        // switch the coordinate of the LIDAR (this shouldn't need switched because the transformation takes care of this, i deleted these lines of code)
	        point.clear(); //should always clear before pushing back if the vector is supposed to be empty before pushing back, previously this was being done after pushing back...
	        point.push_back(average_x);
	        point.push_back(average_y);
	        point.push_back(average_z);
	        point.push_back(variance_z);
	        _local_grid_map.push_back(point);

	        // // this part use the new defined array instead of the old vector of vector mode
	        int x_index = 0;
	        int y_index = 0;
	        x_index = floor(average_x);
	        y_index	= floor(average_y);
	        _GridMap[x_index][y_index].x_mean = average_x;
	        _GridMap[x_index][y_index].y_mean = average_y;
	        _GridMap[x_index][y_index].z_mean = average_z;
	        _GridMap[x_index][y_index].var_z = variance_z;
	        _GridMap[x_index][y_index].occupy = true;
	    }
	}


	//check and determine if the point is ground adjacent or non-adjacent

	//define a cloud to transfer _local_grid_map to pcl format and use its KNN function
	pcl::PointCloud<pcl::PointXYZ>::Ptr adjacent_check (new pcl::PointCloud<pcl::PointXYZ>);

	adjacent_check->width = _local_grid_map.size();
	//cout << _local_grid_map.size() << " is the size of local grid map" << endl;
    adjacent_check->height = 1;
    adjacent_check->points.resize (adjacent_check->width * adjacent_check->height);

	// copy all points from _local_grid_map to the new cloud
	for (int i = 0; i < _local_grid_map.size(); i ++)
	{
		adjacent_check->points[i].x = _local_grid_map[i][0];
    	adjacent_check->points[i].y = _local_grid_map[i][1];
    	adjacent_check->points[i].z = 0.0f;
	}

	// define a KdTree object
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	// set input of the KtTree object
    kdtree.setInputCloud (adjacent_check);

    // define KNN parameters
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // set the search radius to determine if the point is ground adjacent
    float radius = 1.5;

    //cout << adjacent_check->points.size() << " is the size of cloud" << endl;
    // check each point in the cloud
    for (int i = 0; i < adjacent_check->points.size(); i ++)
    {
    	// see how many points are within the range of search
    	kdtree.radiusSearch (adjacent_check->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    	// if there are more than 7 points, that means the particular point is surrounding by many points, so non-ground adjacent
    	if (pointIdxRadiusSearch.size() > 7)
    	{
    		point.clear();
    		point.push_back(_local_grid_map[i][0]);
	        point.push_back(_local_grid_map[i][1]);
	        point.push_back(_local_grid_map[i][2]);
	        point.push_back(_local_grid_map[i][3]);
	        point.push_back(0);
    	}

    	// otherwise is ground adjacent
    	else
    	{
    		point.clear();
    		point.push_back(_local_grid_map[i][0]);
	        point.push_back(_local_grid_map[i][1]);
	        point.push_back(_local_grid_map[i][2]);
	        point.push_back(_local_grid_map[i][3]);
	        point.push_back(1);
    	}

    	// push the info back to _local_grid_map_new
    	_local_grid_map_new.push_back(point);
    }

	//copy filtered point cloud after hard thresholding and ground removal
	_object_filtered = *object_filtered; 
}

void LidarFilter::doMathHoming()
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr object_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	*object_filtered= _object_filtered;

	// FROM HERE, IS THE HOME BEACON CYLINDER DETECTION PART
	// ONLY KEEP POINTS WITHIN THE HOME DETECTION RANGE
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(object_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-5,5); 
    pass.filter(*object_filtered);
    pass.setInputCloud(object_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-home_detection_range,home_detection_range);
    pass.filter(*object_filtered);
    pass.setInputCloud(object_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-home_detection_range,home_detection_range); 
    pass.filter(*object_filtered);
    //ROS_INFO("PassThrough done.");
	
	// std::stringstream ss;
	// ss << "file.pcd";
	// pcl::io::savePCDFile( ss.str(), *object_filtered);

    // CREATING THE KDTREE OBJECT FOR THE SEARCH METHOD OF THE EXTRACTION
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>); //for clustering
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZI> ());
    tree->setInputCloud (object_filtered);
    //cout << "0" << endl;
    
    //use the euclidean clustering method to put points into different clusters based on their relative distance
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.3); // distance threshold
    ec.setMinClusterSize (50); //minial size to generate a cluster 
    ec.setMaxClusterSize (5000); //max size
    ec.setSearchMethod (tree);
    ec.setInputCloud (object_filtered); //use the points after ground removal 
    ec.extract (cluster_indices);

    //cout << "1" << endl;
    
    //generate empty clouds to hold previous cluster
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_middle (new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> points_cluster;
    for (int ii=0;ii<cluster_indices.size ();ii++)
    {
        points_cluster.push_back(cloud_middle);
    }
    
	    //cout << "2" << endl;
	    
    //put above clusters into differnt point clouds
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)   
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
        	cloud_cluster->points.push_back (object_filtered->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        points_cluster[j] = cloud_cluster;
        j++;
    }

    //from here is the cylinder detection method 
    //input data is previous clusters
    std::vector<cylinder> cylinders;
    cylinders.clear();
    //loop through clusters
    //ROS_INFO("%i clusters extracted from scan.", points_cluster.size());
    static bool stopSavingDataToFile = false;
    bool cylinderWasDetected = false;
	std::ofstream outputFile;
	if(stopSavingDataToFile==false)
	{
		outputFile.open(fileName.c_str(), ofstream::out | ofstream::trunc);
	}
	
	//parameters need to run the cylinder detection algorithm
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg;
	pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZI> ());

	//define variables for point intensity value checking
	int high_intensity_counter = 0;
	int high_intensity_threshold = 200;
	bool high_intensity_cluster = false;

    for (int i=0; i<points_cluster.size(); i++)
    {
    	// pre-check if that particular cluser fit the general requirement
    	float max_x_pre = 0;
    	float min_x_pre = 0;
    	float max_y_pre = 0;
    	float min_y_pre = 0;
    	float max_z_pre = 0;
    	float min_z_pre = 0;

    	max_x_pre = points_cluster[i]->points[0].x;
    	min_x_pre = points_cluster[i]->points[0].x;
    	max_y_pre = points_cluster[i]->points[0].y;
    	min_y_pre = points_cluster[i]->points[0].y;
    	max_z_pre = points_cluster[i]->points[0].z;
    	min_z_pre = points_cluster[i]->points[0].z;

    	for (int ii=0; ii<points_cluster[i]->points.size();ii++)
    	{
    		if(points_cluster[i]->points[ii].x > max_x_pre)
    		{
    			max_x_pre = points_cluster[i]->points[ii].x;
    		}
    		if(points_cluster[i]->points[ii].x < min_x_pre)
    		{
    			min_x_pre = points_cluster[i]->points[ii].x;
    		}
    		if(points_cluster[i]->points[ii].y > max_y_pre)
    		{
    			max_y_pre = points_cluster[i]->points[ii].y;
    		}
    		if(points_cluster[i]->points[ii].y < min_y_pre)
    		{
    			min_y_pre = points_cluster[i]->points[ii].y;
    		}
    		if(points_cluster[i]->points[ii].z > max_z_pre)
    		{
    			max_z_pre = points_cluster[i]->points[ii].z;
    		}
    		if(points_cluster[i]->points[ii].z < min_z_pre)
    		{
    			min_z_pre = points_cluster[i]->points[ii].z;
    		}

    		//check if enough high intensity points are detected in one cluster
    		if(points_cluster[i]->points[ii].intensity > high_intensity_threshold)
    		{
    			high_intensity_counter = high_intensity_counter + 1;
    		}

    	}

    	// a cluster needs to have at least 15 high intensity points
    	if (high_intensity_counter > 15)
    	{
    		high_intensity_cluster = true;
    	}

	    //pre check to filter out some obvious false detection
    	if(high_intensity_cluster == true && abs(max_x_pre - min_x_pre) < 1 && abs(max_y_pre - min_y_pre) < 1 && abs(max_z_pre - min_z_pre) >0.5 && abs(max_z_pre - min_z_pre) < 3)
    	{
    		ne.setSearchMethod (tree2);
	        ne.setInputCloud (points_cluster[i]);
	        ne.setKSearch (25); //use 25 nearest points to run the algorithm
	        ne.compute (*cloud_normals);

	        //ROS_INFO("cluster %i has %i points", i, cloud_normals->points.size());

	        // CREATE THE SEGMENTATION OBJECT FOR CYLINDER SEGMENTATION AND SET ALL THE PARAMETERS
	        seg.setOptimizeCoefficients (true);
	        seg.setModelType (pcl::SACMODEL_CYLINDER);
	        seg.setMethodType (pcl::SAC_RANSAC);
	        seg.setNormalDistanceWeight (0.01);
	        seg.setMaxIterations (1000);
	        seg.setDistanceThreshold (0.05);
	        seg.setRadiusLimits (0.14,0.17);
	        seg.setInputCloud (points_cluster[i]);
	        seg.setInputNormals (cloud_normals);

	        // OBTAIN THE CYLINDER INLIERS AND COEFFICIENTS
	        seg.segment (*inliers_cylinder, *coefficients_cylinder);
	        extract.setInputCloud (points_cluster[i]);
	        extract.setIndices (inliers_cylinder);
	        extract.setNegative (false);
	        extract.filter (*cloud_cylinder);
	        //cout << cloud_cylinder->points.size() << " points can be fitted into a cylinder model from cluter " << i << "."<<  endl;
	        
	        // IF 80% POINTS IN A CLUSTER CAN BE FITTED INTO A CYLINDER MODEL, THEN HOME CYLINDER IS DETECTED
	        cylinder current_cylinder;
	        if(float(cloud_cylinder->points.size())/float(cloud_normals->points.size()) >= 0.9)
	        {
	        	cylinderWasDetected = true;
	            ROS_INFO("cluster %i has the size of %i and has fit the cylinder model with probability %f", i, cloud_normals->points.size(), seg.getProbability());
	            cout << "Model coefficients: " << coefficients_cylinder->values[0] << " "
	                                            << coefficients_cylinder->values[1] << " "
	                                            << coefficients_cylinder->values[2] << " "
	                                            << coefficients_cylinder->values[3] << " "
	                                            << coefficients_cylinder->values[4] << " "
	                                            << coefficients_cylinder->values[5] << " "
	                                            << coefficients_cylinder->values[6] << endl;
	            cout << "Probability is " << seg.getProbability () << endl;

	            //test the intensity data
	            //for (int jj=1; jj<cloud_cylinder->points.size();jj++)
	            //{
	            //	cout << cloud_cylinder->points[jj].intensity << endl;
	            //}

				float max_x = cloud_cylinder->points[0].x;
				float max_y = cloud_cylinder->points[0].y;
				float min_x = cloud_cylinder->points[0].x;
				float min_y = cloud_cylinder->points[0].y;
				for (int jj=1; jj<cloud_cylinder->points.size();jj++)
				{
					if(cloud_cylinder->points[jj].x > max_x)
					    max_x = cloud_cylinder->points[jj].x;
					if(cloud_cylinder->points[jj].y > max_y)
					    max_y = cloud_cylinder->points[jj].y;
					if(cloud_cylinder->points[jj].x < min_x)
					    min_x = cloud_cylinder->points[jj].x;
					if(cloud_cylinder->points[jj].y < min_y)
					    min_y = cloud_cylinder->points[jj].y;
			    }
				if(abs(max_x-min_x)<0.6 && abs(max_y-min_y)<0.6)
				{
					current_cylinder.point_in_space(0,0) =  (double)coefficients_cylinder->values[0];
					current_cylinder.point_in_space(1,0) = -(double)coefficients_cylinder->values[1];
					current_cylinder.point_in_space(2,0) = -(double)coefficients_cylinder->values[2];
					current_cylinder.axis_direction(0,0) =  (double)coefficients_cylinder->values[3];
					current_cylinder.axis_direction(1,0) = -(double)coefficients_cylinder->values[4];
					current_cylinder.axis_direction(2,0) = -(double)coefficients_cylinder->values[5];
					current_cylinder.raius_estimate(0,0) =  (double)coefficients_cylinder->values[6];
			    	
					current_cylinder.points.zeros(3,cloud_cylinder->points.size());
					for (int jj=0; jj<cloud_cylinder->points.size(); jj++)
					{
						current_cylinder.points(0,jj)= (double)cloud_cylinder->points[jj].x;
						current_cylinder.points(1,jj)=-(double)cloud_cylinder->points[jj].y;
						current_cylinder.points(2,jj)=-(double)cloud_cylinder->points[jj].z;
					}
					//cout << "current_cylinder.points size = " << current_cylinder.points.n_rows << ", " << current_cylinder.points.n_cols << endl;
					cylinders.push_back(current_cylinder);
				}
	        }
    	}

        
    }
    //begin homing detection from cylinders
    if(cylinders.size()>=2) 
    {
	    // find correct cylinders
	    arma::mat xs1;
	    arma::mat xs2;
	    arma::mat ys1;
	    arma::mat ys2;
	    arma::mat R(3,3); // Roll Matrix
		arma::mat P(3,3); // Pitch Matrix
		arma::mat Rot(3,3); // Total Rotation Matrix
		double dist = 2.0-12.0*0.0254;
		double r = 6.0*0.0254;
		double t, c1_x, c1_y, c2_x, c2_y, x, y, ax1, ay1, ax2, ay2, x_mean, y_mean, d, bearing;
		double v1_x, v1_y, v2_x, v2_y, v1_mag, v2_mag, v_dot, X1s_x, X1s_y, X2s_x, X2s_y, X1s_mag, X2s_mag;
		double cx1, cx2, cy1, cy2;
		bool cylinder_found = false;


		// rotate points and transform cylinder parameters
	    for (int ii=0; ii<cylinders.size(); ii++)
	    {
			if (cylinders[ii].axis_direction(2,0)!=0)
			{
				t = -cylinders[ii].point_in_space(2,0)/cylinders[ii].axis_direction(2,0);
				cylinders[ii].point_in_space(0,0) = cylinders[ii].point_in_space(0,0)+t*cylinders[ii].axis_direction(0,0);
				cylinders[ii].point_in_space(1,0) = cylinders[ii].point_in_space(1,0)+t*cylinders[ii].axis_direction(1,0);
				cylinders[ii].point_in_space(2,0) = 0.0;
			}
	    }

	    //cout << "1" << endl;
	    // find all cylinders correct distance apart
		for (int ii=0; ii<cylinders.size()-1; ii++)
	    {
	    	//cout << "2" << endl;
	    	c1_x = cylinders[ii].point_in_space(0,0);
	    	c1_y = cylinders[ii].point_in_space(1,0);
	    	for (int jj=ii+1; jj<cylinders.size(); jj++)
	    	{
	    		//cout << "3" << endl;
	    		c2_x = cylinders[jj].point_in_space(0,0);
	    		c2_y = cylinders[jj].point_in_space(1,0);
	    		if (abs(sqrt((c1_x-c2_x)*(c1_x-c2_x)+(c1_y-c2_y)*(c1_y-c2_y))-dist)<0.05)
	    		{
	    			cylinder_found = true;
	    			if (c1_x*c2_y-c2_x*c1_y>0)
	    			{
	    				xs1 = cylinders[ii].points.row(0);
	    				ys1 = cylinders[ii].points.row(1);
	    				xs2 = cylinders[jj].points.row(0);
	    				ys2 = cylinders[jj].points.row(1);
						cx1 = c1_x;
						cy1 = c1_y;
						cx2 = c2_x;
						cy2 = c2_y;
	    			}
	    			else
	    			{
	    				xs1 = cylinders[jj].points.row(0);
	    				ys1 = cylinders[jj].points.row(1);
	    				xs2 = cylinders[ii].points.row(0);
	    				ys2 = cylinders[ii].points.row(1);
						cx1 = c2_x;
						cy1 = c2_y;
						cx2 = c1_x;
						cy2 = c1_y;
	    			}
	    		}
	    	}
	    }

	    //cout << "4" << endl;
	    // do the fit
		int n1 = xs1.n_cols;
		int n2 = xs2.n_cols;
		arma::mat X(4,1);
		arma::mat FX(n1+n2+1,1);
		arma::mat J(n1+n2+1,4);
		arma::mat W(n1+n2+1,n1+n2+1,arma::fill::eye);
		arma::mat JtWJ(4,4);
		W(n1+n2,n1+n2) = 1600;
		bool explode = false;

		if (cylinder_found)
		{
			X(0,0) = cx1; //column 1 x-center
			X(1,0) = cy1; //column 1 y-center
			X(2,0) = cx2; //column 2 x-center
			X(3,0) = cy2; //column 2 y-center
			/*//ROS_INFO("Cylinder centroids = %f, %f, %f, %f", c1_x, c1_y, c2_x, c2_y);
			// alternate initial guess
			double X1s_x = arma::as_scalar(arma::mean(xs1,1));
			double X1s_y = arma::as_scalar(arma::mean(ys1,1));
			double X2s_x = arma::as_scalar(arma::mean(xs2,1));
			double X2s_y = arma::as_scalar(arma::mean(ys2,1));
			// cout<<"X1s_x="<<X1s_x<<endl;
			// cout<<"X1s_y="<<X1s_y<<endl;
			// cout<<"X2s_x="<<X2s_x<<endl;
			// cout<<"X2s_y="<<X2s_y<<endl;
			double X1s_mag = sqrt(X1s_x*X1s_x+X1s_y*X1s_y);
			double X2s_mag = sqrt(X2s_x*X2s_x+X2s_y*X2s_y);
			X1s_x = X1s_x+r*X1s_x/X1s_mag;
			X1s_y = X1s_y+r*X1s_y/X1s_mag;
			X2s_x = X2s_x+r*X2s_x/X2s_mag;
			X2s_y = X2s_y+r*X2s_y/X2s_mag;
			X(0) = X1s_x; //column 1 x-center
			X(1) = X1s_y; //column 1 y-center
			X(2) = X2s_x; //column 2 x-center
			X(3) = X2s_y; //column 2 y-center*/

			for (int ii = 0; ii<20; ii++)
			{
				ax1 = X(0,0);
				ay1 = X(1,0);
				ax2 = X(2,0);
				ay2 = X(3,0);
				for (int jj = 0; jj<n1; jj++)
				{
					x = xs1(0,jj);
					y = ys1(0,jj);
					FX(jj,0) = sqrt((x-ax1)*(x-ax1)+(y-ay2)*(y-ay1))-r; 
					J(jj,0) = (ax1-x)/sqrt((ax1-x)*(ax1-x)+(ay1-y)*(ay1-y));
					J(jj,1) = (ay1-y)/sqrt((ax1-x)*(ax1-x)+(ay1-y)*(ay1-y));
				}
				for (int jj = n1; jj<n1+n2; jj++)
				{
					x = xs2(0,jj-n1);
					y = ys2(0,jj-n1);
					FX(jj,0) = sqrt((x-ax2)*(x-ax2)+(y-ay2)*(y-ay2))-r; 
					J(jj,2) = (ax2-x)/sqrt((ax2-x)*(ax2-x)+(ay2-y)*(ay2-y));
					J(jj,3) = (ay2-y)/sqrt((ax2-x)*(ax2-x)+(ay2-y)*(ay2-y));
				}
				FX(n1+n2,0) = sqrt((ax1-ax2)*(ax1-ax2)+(ay1-ay2)*(ay1-ay2))-dist;
				J(n1+n2,0) = (ax1-ax2)/sqrt((ax1-ax2)*(ax1-ax2)+(ay1-ay2)*(ay1-ay2));
				J(n1+n2,1) = (ay1-ay2)/sqrt((ax1-ax2)*(ax1-ax2)+(ay1-ay2)*(ay1-ay2));
				J(n1+n2,2) = (ax2-ax1)/sqrt((ax1-ax2)*(ax1-ax2)+(ay1-ay2)*(ay1-ay2));
				J(n1+n2,3) = (ay2-ay1)/sqrt((ax1-ax2)*(ax1-ax2)+(ay1-ay2)*(ay1-ay2));
				JtWJ = J.st()*W*J;
				if (!JtWJ.has_nan() && !JtWJ.has_inf())
				{
					X = X-0.25*arma::solve(JtWJ,J.st()*W*FX);
				}
				else
				{
					explode = true;
				}
			}

			//cout << "5" << endl;
			// this part is used for comparing the results between two parts
			/*x_mean = (X(0,0)+X(2,0))/2;
			y_mean = (X(1,0)+X(3,0))/2;

			d = sqrt(x_mean*x_mean+y_mean*y_mean);

			v2_x = X(0,0)-X(2,0);
			v2_y = X(1,0)-X(3,0);

			v1_mag = sqrt(x_mean*x_mean+y_mean*y_mean); 
			v2_mag = sqrt(v2_x*v2_x+v2_y*v2_y); 
			v1_x = x_mean/v1_mag;
			v1_y = y_mean/v1_mag;
			v2_x = v2_x/v2_mag;
			v2_y = v2_y/v2_mag;

			v_dot = v1_x*v2_x+v1_y*v2_y;
			bearing = acos(v_dot)-3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798214808651/2;
			double x_est_k = d*cos(bearing);
			double y_est_k = d*sin(bearing);
			double b_h_diff_k = atan2(-v1_y,-v1_x); 
			double heading_est_k = -(b_h_diff-bearing);*/



			x_mean = (cx1+cx2)/2;
			y_mean = (cy1+cy2)/2;
			d = sqrt(x_mean*x_mean+y_mean*y_mean);

			v2_x = cx1-cx2;
			v2_y = cy1-cy2;
			v1_mag = sqrt(x_mean*x_mean+y_mean*y_mean); 
			v2_mag = sqrt(v2_x*v2_x+v2_y*v2_y); 
			v1_x = x_mean/v1_mag;
			v1_y = y_mean/v1_mag;
			v2_x = v2_x/v2_mag;
			v2_y = v2_y/v2_mag;

			v_dot = v1_x*v2_x+v1_y*v2_y;
			bearing = acos(v_dot)-3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798214808651/2;
			double x_est = d*cos(bearing);
			double y_est = d*sin(bearing);
			double b_h_diff = atan2(-v1_y,-v1_x); 
			double heading_est = -(b_h_diff-bearing);

			ROS_INFO("\nHOMING UPDATE!");
			ROS_INFO("x = %f", x_est);
			ROS_INFO("y = %f", y_est);
			ROS_INFO("heading = %f", heading_est*180.0/3.14159265);

			_homing_x=x_est;
			_homing_y=y_est;
			_homing_heading=heading_est;
			_homing_found=true;
			ROS_INFO("********************");
			ROS_INFO("x_est = %f",x_est);
			ROS_INFO("y_est = %f",y_est);
			ROS_INFO("x_mean = %f",x_mean);
			ROS_INFO("y_mean = %f",y_mean);
			ROS_INFO("heading = %f\n",heading_est);
			ROS_INFO("explode = %f\n",explode);
		}
		else
		{
			_homing_x=0;
			_homing_y=0;
			_homing_heading=0;
			_homing_found=false;
		} //end if cylinder found
		double diff1, diff2;
		diff1 = sqrt((cx1-X(0,0))*(cx1-X(0,0))+(cy1-X(1,0))*(cy1-X(1,0)));
		diff2 = sqrt((cx2-X(2,0))*(cx2-X(2,0))+(cy2-X(3,0))*(cy2-X(3,0)));

		if(stopSavingDataToFile==false && _homing_found==true && (diff1+diff2<0.3 || explode == true))
		{
			for (int i=0; i<points_cluster.size(); i++)
			{
				for (int jj=0; jj<cloud_cylinder->points.size(); jj++)
				{
					outputFile << cylinders[i].points(0,jj) << ",";
					outputFile << cylinders[i].points(1,jj) << ",";
					outputFile << cylinders[i].points(2,jj) << ",";
					outputFile << i << ","; //cylinder number
					outputFile << cylinders[i].point_in_space(0,0) << ",";
					outputFile << cylinders[i].point_in_space(1,0) << ",";
					outputFile << cylinders[i].point_in_space(2,0) << ",";
					outputFile << cylinders[i].axis_direction(0,0) << ",";
					outputFile << cylinders[i].axis_direction(1,0) << ",";
					outputFile << cylinders[i].axis_direction(2,0) << ",";
					outputFile << cylinders[i].raius_estimate(0,0);
					outputFile << std::endl; 	
				}
			}
			stopSavingDataToFile=true; 
		}
		else
		{
			outputFile.close();
		}
	}
}


//use this member function to detect homing cylinder from a long distance 10 m < distance < 20 m
void LidarFilter::doLongDistanceHoming()
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr object_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	*object_filtered= _object_filtered;

	// ONLY KEEP POINTS WITHIN THE HOME DETECTION RANGE
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(object_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-10,10); //long distance detection, may have slanted ground 
    pass.filter(*object_filtered);
    pass.setInputCloud(object_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-home_detection_range,home_detection_range);
    pass.filter(*object_filtered);
    pass.setInputCloud(object_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-home_detection_range,home_detection_range); 
    pass.filter(*object_filtered);
    //ROS_INFO("PassThrough done.");
	
	// std::stringstream ss;
	// ss << "file.pcd";
	// pcl::io::savePCDFile( ss.str(), *object_filtered);

    // CREATING THE KDTREE OBJECT FOR THE SEARCH METHOD OF THE EXTRACTION
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>); //for clustering
    tree->setInputCloud (object_filtered);
    //cout << "0" << endl;
    
    //use the euclidean clustering method to put points into different clusters based on their relative distance
    //parameters are different than close ditance detection
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (1.0); // distance threshold
    ec.setMinClusterSize (15); //minial size to generate a cluster 
    ec.setMaxClusterSize (5000); //max size
    ec.setSearchMethod (tree);
    ec.setInputCloud (object_filtered); //use the points after ground removal 
    ec.extract (cluster_indices);

    //cout << "1" << endl;
    
    //generate empty clouds to hold previous cluster
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_middle (new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> points_cluster;
    for (int ii=0;ii<cluster_indices.size ();ii++)
    {
        points_cluster.push_back(cloud_middle);
    }
    
	//cout << "2" << endl;
	    
    //put above clusters into differnt point clouds
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)   
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
        	cloud_cluster->points.push_back (object_filtered->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        points_cluster[j] = cloud_cluster;
        j++;
    }

	//define variables for point intensity value checking
	int high_intensity_counter = 0;
	int high_intensity_threshold = 200;
	bool high_intensity_cluster = false;

    for (int i=0; i<points_cluster.size(); i++)
    {
    	// pre-check if that particular cluser fit the general requirement
    	float max_x_pre = 0;
    	float min_x_pre = 0;
    	float max_y_pre = 0;
    	float min_y_pre = 0;
    	float max_z_pre = 0;
    	float min_z_pre = 0;
    	float mean_x_cluster_pre = 0; //mean position of the x position high intensity cylinder
    	float mean_y_cluster_pre = 0; //mean position of the y position high intensity cylinder

    	max_x_pre = points_cluster[i]->points[0].x;
    	min_x_pre = points_cluster[i]->points[0].x;
    	max_y_pre = points_cluster[i]->points[0].y;
    	min_y_pre = points_cluster[i]->points[0].y;
    	max_z_pre = points_cluster[i]->points[0].z;
    	min_z_pre = points_cluster[i]->points[0].z;

    	for (int ii=0; ii<points_cluster[i]->points.size();ii++)
    	{
    		if(points_cluster[i]->points[ii].x > max_x_pre)
    		{
    			max_x_pre = points_cluster[i]->points[ii].x;
    		}
    		if(points_cluster[i]->points[ii].x < min_x_pre)
    		{
    			min_x_pre = points_cluster[i]->points[ii].x;
    		}
    		if(points_cluster[i]->points[ii].y > max_y_pre)
    		{
    			max_y_pre = points_cluster[i]->points[ii].y;
    		}
    		if(points_cluster[i]->points[ii].y < min_y_pre)
    		{
    			min_y_pre = points_cluster[i]->points[ii].y;
    		}
    		if(points_cluster[i]->points[ii].z > max_z_pre)
    		{
    			max_z_pre = points_cluster[i]->points[ii].z;
    		}
    		if(points_cluster[i]->points[ii].z < min_z_pre)
    		{
    			min_z_pre = points_cluster[i]->points[ii].z;
    		}

    		//sum up
    		mean_x_cluster_pre = mean_x_cluster_pre + points_cluster[i]->points[ii].x;
    		mean_y_cluster_pre = mean_y_cluster_pre + points_cluster[i]->points[ii].y;

    		//check if enough high intensity points are detected in one cluster
    		if(points_cluster[i]->points[ii].intensity > high_intensity_threshold)
    		{
    			high_intensity_counter = high_intensity_counter + 1;
    		}

    	}

    	//calculate the mean value
    	mean_x_cluster_pre =  mean_x_cluster_pre/points_cluster[i]->points.size();
    	mean_y_cluster_pre =  mean_y_cluster_pre/points_cluster[i]->points.size();

    	// a cluster needs to have at least 15 high intensity points
    	if (high_intensity_counter > 10)
    	{
    		high_intensity_cluster = true;
    	}

    	if(high_intensity_cluster == true && abs(max_x_pre - min_x_pre) < 1 && abs(max_y_pre - min_y_pre) < 1 && abs(max_z_pre - min_z_pre) >0.5 && abs(max_z_pre - min_z_pre) < 3)
    	{
    		ROS_INFO_STREAM("The cluser is located at " << mean_x_cluster_pre << ", " << mean_y_cluster_pre);

    		//check the nearest big cluster
    	    for (int k=0; k<points_cluster.size(); k++)
    	    {
    	    	if (k != i)
    	    	{
    	    		//variables to define the 2nd cluster
    	    		float max_x_post = 0;
			    	float min_x_post = 0;
			    	float max_y_post = 0;
			    	float min_y_post = 0;
			    	float max_z_post = 0;
			    	float min_z_post = 0;
			    	float mean_x_cluster_post = 0; //mean position of the x position of the closest large cluster
			    	float mean_y_cluster_post = 0; //mean position of the y position of the closest large cluster

			    	max_x_post = points_cluster[k]->points[0].x;
			    	min_x_post = points_cluster[k]->points[0].x;
			    	max_y_post = points_cluster[k]->points[0].y;
			    	min_y_post = points_cluster[k]->points[0].y;

			    	for (int ii=0; ii<points_cluster[k]->points.size();ii++)
			    	{
			    		if(points_cluster[k]->points[ii].x > max_x_post)
			    		{
			    			max_x_post = points_cluster[k]->points[ii].x;
			    		}
			    		if(points_cluster[k]->points[ii].x < min_x_post)
			    		{
			    			min_x_post = points_cluster[k]->points[ii].x;
			    		}
			    		if(points_cluster[k]->points[ii].y > max_y_post)
			    		{
			    			max_y_post = points_cluster[k]->points[ii].y;
			    		}
			    		if(points_cluster[k]->points[ii].y < min_y_post)
			    		{
			    			min_y_post = points_cluster[k]->points[ii].y;
			    		}
			    		if(points_cluster[k]->points[ii].z > max_z_post)
			    		{
			    			max_z_post = points_cluster[k]->points[ii].z;
			    		}
			    		if(points_cluster[k]->points[ii].z < min_z_post)
			    		{
			    			min_z_post = points_cluster[k]->points[ii].z;
			    		}

			    		//sum up
			    		mean_x_cluster_post = mean_x_cluster_post + points_cluster[k]->points[ii].x;
			    		mean_y_cluster_post = mean_y_cluster_post + points_cluster[k]->points[ii].y;
			    	}

			    	//calculate the mean value
			    	mean_x_cluster_post =  mean_x_cluster_post/points_cluster[k]->points.size();
			    	mean_y_cluster_post =  mean_y_cluster_post/points_cluster[k]->points.size();

			    	//calculate their relative distance
			    	float distance = 0;
			    	distance = sqrt((mean_x_cluster_pre - mean_x_cluster_post)*(mean_x_cluster_pre - mean_x_cluster_post)
			    		+ (mean_y_cluster_pre - mean_y_cluster_post)*(mean_y_cluster_pre - mean_y_cluster_post));

			    	//ROS_INFO_STREAM("Distance is " << distance << " .");
			    	if(int(distance) <= 2 && abs(max_x_post - min_x_post) < 1.5 && abs(max_y_post - min_y_post) < 1.5 && abs(max_z_pre - min_z_pre) >0.5)
			    	{
			    		ROS_INFO_STREAM("Distance between two clusters is " << distance << " .");
			    	}
    	    	}
    	    }
    	}
    	high_intensity_cluster = false;
    	high_intensity_counter = 0;
    }
}
