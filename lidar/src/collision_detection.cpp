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
	_sub_waypoint = _nh.subscribe("/control/exec/nextwaypoint", 1, &CollisionDetection::waypointsCallback, this);
	_sub_position = _nh.subscribe("/hsm/masterexec/globalpose", 1, &CollisionDetection::positionCallback, this);

	//predictive avoidance service
	returnHazardMapServ = _nh.advertiseService("/lidar/collisiondetection/createroihazardmap", &CollisionDetection::returnHazardMap, this);
	
	//collision output
	_collision_status = 0;
}

void CollisionDetection::Initializations()
{
	//parameters
	short_distance = 3;
	long_distance = 5;
	threshold_obstacle_distance = 0.5;
	threshold_obstacle_number = 0;
	threshold_min_angle = 45; //degree, min angle to turn

	error_angle = 11 * PI / 180;	//turn more 10 degree, one more for floor
}

void CollisionDetection::waypointsCallback(messages::NextWaypointOut const &waypoint_msg)
{
	_xg = waypoint_msg.globalX;
	_yg = waypoint_msg.globalY;
}

void CollisionDetection::positionCallback(messages::RobotPose const &position_msg)
{
	_xposition = position_msg.x;
	_yposition = position_msg.y;
	_headingposition = position_msg.heading * PI / 180;	//should change to radian
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
    T_temporary(0,3) = -0.45;	//translate from lidar to robot center

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
	msg.distance_to_drive = _distance_to_drive;
	msg.angle_to_drive = _angle_to_drive;
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

	//angle for robot to turn
	std::vector<double> angle;
	angle.clear();
	
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
					//check how many degree the robot should turn
					// angle.push_back((double)atan2(cloud->points[i].x,cloud->points[i].y));	//radian
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

	ROS_INFO_STREAM("collision_point_counter: " << collision_point_counter);

	//check if points exceed threshold
	if(collision_point_counter > _TRIGGER_POINT_THRESHOLD)
	{	
		_collision_status = 1;	//detected a obstacle

		int choice;	//0: big long; 1: big short; 2: small long; 3: small short; 4: no option; x0: normal; x1: one side
		int choice_angle; //00: normal big; 10: normalsmall; 01: one side big; 11: one side small
		int choice_big_long = 0;
		int choice_big_short = 0;
		int choice_small_long = 0;
		int choice_small_short = 0;

		double big_angle;
		double small_angle;

		double xg_local;
		double yg_local;

		//get local coordinate
		xg_local = _xg * cos(_headingposition) + _yg * sin(_headingposition) + _xposition;
		yg_local = -1 * _xg * sin(_headingposition) + _yg * sin(_headingposition) + _yposition;

		// xg_local = 5 + _xposition;
		// yg_local = 5 + _yposition;

		
		//use hazard map to detect angle
		generateAvoidancemap();

		//test
		// generateHazardmap();

		for(int i = 0; i < _hazard_x.size(); i++)
		{
			if(_hazard_x[i] < 5 && _hazard_x[i] > 0 && _hazard_y[i] < 1 && _hazard_y[i] > -1)
			{
				angle.push_back((double)atan2(_hazard_x[i],_hazard_y[i]));	//radian

			}
		}
		
		//sort angles
		if(angle.size() > 1)
		{
			std::sort(angle.begin(), angle.end());	//increase
		
			// std::sort(angle.begin(), angle.end());	//decrise

			big_angle = angle[angle.size() - 1];
			small_angle = angle[0];

			ROS_INFO_STREAM("big_angle: " << big_angle * 180 / PI << " small_angle: " << small_angle * 180 / PI);

			//check turn angle
			if(90 - (big_angle * 180 / PI) > 45)
			{
				big_angle = 91 * PI / 180;
			}
			else if(90 - (big_angle * 180 / PI) <= 45 && 90 - (big_angle * 180 / PI) > 0)
			{
				big_angle = (90 + 45) * PI / 180;
			}
			else if(90 - (big_angle * 180 / PI) <= 0)
			{
				big_angle = big_angle + 45 * PI / 180;
			}

			if(90 - (small_angle * 180 / PI) < -45)
			{
				small_angle = 89 * PI / 180;
			}
			else if(90 - (small_angle * 180 / PI) >= -45 && 90 - (small_angle * 180 / PI) < 0)
			{
				small_angle = 45 * PI / 180;
			}
			else if(90 - (small_angle * 180 / PI) >= 0)
			{
				small_angle = small_angle - 45 * PI / 180;
			}
			ROS_INFO_STREAM("after big_angle: " << big_angle * 180 / PI << " after small_angle: " << small_angle * 180 / PI);
			// big_angle = big_angle + error_angle;
			// small_angle = small_angle - error_angle;

			// if(big_angle * 180 / PI == 90)
			// {
			// 	big_angle = big_angle + 0.017;
			// } 

			// if(small_angle * 180 / PI == 90)
			// {
			// 	small_angle = small_angle - 0.017;
			// } 

			// generateAvoidancemap();

			int count_big_long_first = -1;
			int count_big_short_first = -1;
			int count_small_long_first = -1;
			int count_small_short_first = -1;
			int count_big_long_second = -1;
			int count_big_short_second = -1;
			int count_small_long_second = -1;
			int count_small_short_second = -1;

			count_big_long_first = firstChoice(big_angle, long_distance);
			ROS_INFO_STREAM("count_big_long_first: " << count_big_long_first);
			// count_big_short_first = firstChoice(big_angle, short_distance);
			count_small_long_first = firstChoice(small_angle, long_distance);
			ROS_INFO_STREAM("count_small_long_first: " << count_small_long_first);
			// count_small_short_first = firstChoice(small_angle, short_distance);

			if(count_big_long_first == threshold_obstacle_number)
			{
				count_big_long_second = secondChoice(big_angle, long_distance, xg_local, yg_local);
				count_big_short_second = secondChoice(big_angle, short_distance, xg_local, yg_local);

				if(count_big_long_second == threshold_obstacle_number)
				{
					choice_big_long = 1;
				}

				if(count_big_short_second == threshold_obstacle_number)
				{
					choice_big_short = 1;
				}
			}
			else if(count_big_long_first > threshold_obstacle_number)
			{
				count_big_short_first = firstChoice(big_angle, short_distance);

				if(count_big_short_first == threshold_obstacle_number)
				{
					count_big_short_second = secondChoice(big_angle, short_distance, xg_local, yg_local);
					if(count_big_short_second == threshold_obstacle_number)
					{
						choice_big_short = 1;
					}
				}
			}

			if(count_small_long_first == threshold_obstacle_number)
			{
				count_small_long_second = secondChoice(small_angle, long_distance, xg_local, yg_local);
				count_small_short_second = secondChoice(small_angle, short_distance, xg_local, yg_local);

				if(count_small_long_second == threshold_obstacle_number)
				{
					choice_small_long = 1;
				}

				if(count_small_short_second == threshold_obstacle_number)
				{
					choice_small_short = 1;
				}
			}
			else if(count_small_long_first > threshold_obstacle_number)
			{
				count_small_short_first = firstChoice(small_angle, short_distance);

				if(count_small_short_first == threshold_obstacle_number)
				{
					count_small_short_second = secondChoice(small_angle, short_distance, xg_local, yg_local);
					if(count_small_short_second == threshold_obstacle_number)
					{
						choice_small_short = 1;
					}
				}
			}

			ROS_INFO_STREAM("count_big_long_second: " << count_big_long_second);
			ROS_INFO_STREAM("count_big_short_second: " << count_big_short_second);
			ROS_INFO_STREAM("count_small_long_second: " << count_small_long_second);
			ROS_INFO_STREAM("count_small_short_second: " << count_small_short_second);
			ROS_INFO_STREAM("xg_local: " << xg_local);
			ROS_INFO_STREAM("yg_local: " << yg_local);

			//calculate angle the robot should turn
			// if(big_angle * 180 / PI > 90 && small_angle * 180 / PI < 90)
			// {
				if(fabs(big_angle * 180 / PI - 90) > fabs(90 - small_angle * 180 / PI))
				{
					choice_angle = 10;
				}
				else
				{
					choice_angle = 00;
				}
			// }
			// else if(big_angle * 180 / PI < 90)
			// {
			// 	choice_angle = 01;
			// }
			// else if(small_angle * 180 / PI > 90)
			// {
			// 	choice_angle = 11;
			// }

			//make a decision
			// if(choice_angle == 00 && choice_big_short == 1)
			// {
			// 	choice = 10;	//big short normal
			// }
			// else if(choice_angle == 10 && choice_small_short == 1)
			// {
			// 	choice = 30;	//small short normal
			// }
			// else if(choice_angle == 00 && choice_big_long == 1)
			// {
			// 	choice = 00;	//big long normal
			// }
			// else if(choice_angle == 10 && choice_small_long == 1)
			// {
			// 	choice = 20;	//small long normal
			// }
			// else if(choice_angle == 01 && choice_big_short == 1)
			// {
			// 	choice = 11;	//big short one side
			// }
			// else if(choice_angle == 11 && choice_small_short == 1)
			// {
			// 	choice = 31;	//small short one side
			// }
			// else if(choice_angle == 01 && choice_big_long == 1)
			// {
			// 	choice = 01;	//big long one side
			// }
			// else if(choice_angle == 11 && choice_small_long == 1)
			// {
			// 	choice = 21;	//small long one side
			// }
			// else
			// {
			// 	choice = 4; //no option
			// }

			if(choice_big_short == 1 && choice_small_short == 1)
			{
				if(choice_angle == 00)
				{
					choice = 10; //big short normal
				}
				else if(choice_angle == 10)
				{
					choice = 30; //small short normal
				}
			}
			else if(choice_big_short == 1)
			{
				choice = 10; //big short normal
			}
			else if(choice_small_short == 1)
			{
				choice = 30;
			}
			else if(choice_big_long == 1 && choice_small_long == 1)
			{
				if(choice_angle == 00)
				{
					choice = 00; //big long normal
				}
				else if(choice_angle == 10)
				{
					choice = 20; //small long normal
				}
			}
			else if(choice_big_long == 1)
			{
				choice = 00; //big long normal
			}
			else if(choice_small_long == 1)
			{
				choice = 20; //small long normal
			}
			else
			{
				choice = 4;	//no options
			}

			// ROS_INFO_STREAM("choice_angle: " << choice_angle);
			//save angle and distance for publishing
			// if(choice == 10)
			// {
			// 	_angle_to_drive = floor(90 - big_angle * 180 / PI);
				
			// 	if(fabs(_angle_to_drive) < threshold_min_angle)
			// 	{
			// 		_angle_to_drive = _angle_to_drive / fabs(_angle_to_drive) * threshold_min_angle;
			// 	}

			// 	_distance_to_drive = short_distance;			
			// }
			// else if(choice == 30)
			// {
			// 	_angle_to_drive = floor(90 - small_angle * 180 / PI);

			// 	if(fabs(_angle_to_drive) < threshold_min_angle)
			// 	{
			// 		_angle_to_drive = _angle_to_drive / fabs(_angle_to_drive) * threshold_min_angle;
			// 	}

			// 	_distance_to_drive = short_distance;
			// }
			// else if(choice == 00)
			// {
			// 	_angle_to_drive = floor(90 - big_angle * 180 / PI);

			// 	if(fabs(_angle_to_drive) < threshold_min_angle)
			// 	{
			// 		_angle_to_drive = _angle_to_drive / fabs(_angle_to_drive) * threshold_min_angle;
			// 	}

			// 	_distance_to_drive = long_distance;
			// }
			// else if(choice == 20)
			// {
			// 	_angle_to_drive = floor(90 - small_angle * 180 / PI);

			// 	if(fabs(_angle_to_drive) < threshold_min_angle)
			// 	{
			// 		_angle_to_drive = _angle_to_drive / fabs(_angle_to_drive) * threshold_min_angle;
			// 	}

			// 	_distance_to_drive = long_distance;
			// }
			// else if(choice == 11)
			// {
			// 	_angle_to_drive = 0;
			// 	_distance_to_drive = short_distance;			
			// }
			// else if(choice == 31)
			// {
			// 	_angle_to_drive = 0;
			// 	_distance_to_drive = short_distance;
			// }
			// else if(choice == 01)
			// {
			// 	_angle_to_drive = 0;
			// 	_distance_to_drive = long_distance;
			// }
			// else if(choice == 21)
			// {
			// 	_angle_to_drive = 0;
			// 	_distance_to_drive = long_distance;
			// }
			// else if(choice == 4)
			// {
			// 	_collision_status = 1;	//no option

			// 	if(yg_local > 0)
			// 	{
			// 		_angle_to_drive = 100;
			// 	}
			// 	else
			// 	{
			// 		_angle_to_drive = -100;
			// 	}
			// 	_distance_to_drive = 5; //no option, turn 100 degree near to the waypoint and drive 5 m

			// 	ROS_INFO_STREAM("No good options");
				
			// }

			if(choice == 10)
			{
				_angle_to_drive = floor(90 - big_angle * 180 / PI);
				
				if(fabs(_angle_to_drive) < 2)
				{
					_angle_to_drive = 0;
				}

				_distance_to_drive = short_distance;			
			}
			else if(choice == 30)
			{
				_angle_to_drive = floor(90 - small_angle * 180 / PI);

				if(fabs(_angle_to_drive) < 2)
				{
					_angle_to_drive = 0;
				}

				_distance_to_drive = short_distance;
			}
			else if(choice == 00)
			{
				_angle_to_drive = floor(90 - big_angle * 180 / PI);

				if(fabs(_angle_to_drive) < 2)
				{
					_angle_to_drive = 0;
				}

				_distance_to_drive = long_distance;
			}
			else if(choice == 20)
			{
				_angle_to_drive = floor(90 - small_angle * 180 / PI);

				if(fabs(_angle_to_drive) < 2)
				{
					_angle_to_drive = 0;
				}

				_distance_to_drive = long_distance;
			}
			// else if(choice == 11)
			// {
			// 	_angle_to_drive = 0;
			// 	_distance_to_drive = short_distance;			
			// }
			// else if(choice == 31)
			// {
			// 	_angle_to_drive = 0;
			// 	_distance_to_drive = short_distance;
			// }
			// else if(choice == 01)
			// {
			// 	_angle_to_drive = 0;
			// 	_distance_to_drive = long_distance;
			// }
			// else if(choice == 21)
			// {
			// 	_angle_to_drive = 0;
			// 	_distance_to_drive = long_distance;
			// }
			else if(choice == 4)
			{
				_collision_status = 1;	//no option

				if(yg_local > 0)
				{
					_angle_to_drive = 100;
				}
				else
				{
					_angle_to_drive = -100;
				}
				_distance_to_drive = 5; //no option, turn 100 degree near to the waypoint and drive 5 m

				ROS_INFO_STREAM("No good options");
				
			}

			ROS_INFO_STREAM("Turn " << _angle_to_drive << " degree and drive " << _distance_to_drive << " m");

			// //write to a function
			// //detect if there are any points near the path
			// double angle;
			// double distance;

			// int count = 0;
			// for(int i = 0; i < _hazard_x.size(); i++)
			// {
			// 	if(_hazard_x[i] > - 1 / tan(angle) * _hazard_y[i] && _hazard_x[i] < - 1 / tan(angle) * _hazard_y[i] + distance / sin(angle))
			// 	{
			// 		if(fabs(cos(angle) * (_hazard_x[i] - _hazard_y[i] * tan(angle))) < threshold_obstacle_distance)
			// 		{
			// 			count++;
			// 		}
			// 	}			
			// }

			// double xg, yg; // waypoint goal
			// if(xg > distance * cos(angle))	//waypoint on top of temp point
			// {
			// 	if(_hazerd_x[i] > distance * cos(angle))
			// 	{
			// 		if(fabs(_hazerd_x[i] - ((distance * sin(angle) - xg) / (distance * cos(angle) - yg)) * _hazard_y[i] - (xg * distance * cos(angle) - yg * distance * sin(angle)) / (distance * cos(angle) - yg)) /
			// 		sqrt(1 + pow((distance * sin(angle) - xg) / (distance * cos(angle) - yg),2)) < threshold_obstacle_distance)
			// 		{
			// 			count++;
			// 		}
			// 	}
			// }
			// else
			// {
			// 	if(_hazerd_x[i] < distance * cos(angle))
			// 	{
			// 		if(fabs(_hazerd_x[i] - ((distance * sin(angle) - xg) / (distance * cos(angle) - yg)) * _hazard_y[i] - (xg * distance * cos(angle) - yg * distance * sin(angle)) / (distance * cos(angle) - yg)) /
			// 		sqrt(1 + pow((distance * sin(angle) - xg) / (distance * cos(angle) - yg),2)) < threshold_obstacle_distance)
			// 		{
			// 			count++;
			// 		}
			// 	}
			// }

			//detect if the left angle bigger than 90 degree

			//detect if the right angle bigger than 90 degree

			// //determine side of collision
			// if(collision_left_counter > collision_right_counter)
			// {
			// 	_collision_status = 1;
			// 	ROS_INFO("COLLISION ON LEFT");
			// 	return 1;
			// }
			// else
			// {
			// 	_collision_status = 2;
			// 	ROS_INFO("COLLISION ON RIGHT");		
			// 	return 2;		
			// }
		}
		else
		{
			if(collision_left_counter > collision_right_counter)
			{
				_angle_to_drive = 45;
			}
			else
			{
				_angle_to_drive = -45;
			}
			
			_distance_to_drive = short_distance;

			ROS_INFO_STREAM("there is no obstacle on hazard map, turn " << _angle_to_drive << " degree and 3 m");
		}
	}
	else
	{
		_collision_status = 0;
		_distance_to_drive = 0;
		_angle_to_drive = 0;
		ROS_INFO("No Collision...");
		return 0;
	}
}

// int CollisionDetection::doMathRANSAC() // SECOND LAYER: RANSAC FIT A PLANE
// {

// }

// 20 * 10 avoidance map
void CollisionDetection::generateAvoidancemap()
{

	//trigger the hazard map generation function
	pcl::PointCloud<pcl::PointXYZI>::Ptr hazard_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    *hazard_cloud = _input_cloud;

    int hazard_map_size_x_pos = 10; 
    int hazard_map_size_x_neg = 5;
    int hazard_map_size_y = 10;	// 2 * 10 both sides

	//remove points based on hard thresholds (too far, too high, too low)
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(hazard_cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-hazard_map_size_x_neg,hazard_map_size_x_pos);
	pass.filter(*hazard_cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-hazard_map_size_y,hazard_map_size_y);
	pass.filter(*hazard_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-5,5); //positive z is down, negative z is up
	pass.filter(*hazard_cloud);


	//create segmentation object for fitting a plane to points in the full cloud using RANSAC (assuming the fit plane represents the ground)
	//pcl::SACSegmentation<pcl::PointXYZI> plane;
	//plane.setOptimizeCoefficients (true); //optional (why is this optional??)
	//plane.setModelType (pcl::SACMODEL_PLANE);
	//plane.setMethodType (pcl::SAC_RANSAC);
	//plane.setMaxIterations (1000); //max iterations for RANSAC

	//******************************
	//the general idea of this part is similar as local map generation, the difference is that the RANSAC fitting is more loose which will put 
	//more points to the ground, therefore the object cluster (hazard cluster) only has real hazarad, therefore, the false alarm rate will reduce
	//In other words, hazard map and path planning only consider real big hazard while small obstacle should be leave to the pure reactive layer
	//******************************

	//plane.setDistanceThreshold (0.75); //ground detection threshold parameter
	//plane.setInputCloud (hazard_cloud); //was raw_cloud

	//segment the points fitted to the plane using ransac
	pcl::SACSegmentation<pcl::PointXYZI> seg_plane;
	seg_plane.setOptimizeCoefficients (true); //optional (why is this optional??)
	seg_plane.setModelType (pcl::SACMODEL_PLANE);
	seg_plane.setMethodType (pcl::SAC_RANSAC);
	seg_plane.setMaxIterations (1000); //max iterations for RANSAC
	seg_plane.setDistanceThreshold (0.75); //ground detection threshold parameter
	seg_plane.setInputCloud (hazard_cloud); //was raw_cloud

	pcl::ModelCoefficients::Ptr coefficients_hazard (new pcl::ModelCoefficients ()); 
	pcl::PointIndices::Ptr inliers_hazard (new pcl::PointIndices ()); 
	seg_plane.segment (*inliers_hazard, *coefficients_hazard);

	
	//seperate the ground points and the points above the ground (object points)
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud (hazard_cloud);
	extract.setIndices (inliers_hazard);

	extract.setNegative (true);
	pcl::PointCloud<pcl::PointXYZI>::Ptr hazard_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	extract.filter (*hazard_filtered);

    //define variables used in this section
	std::vector<float> point;
	std::vector<std::vector<std::vector<float> > > hazard_map_cells((hazard_map_size_x_pos + hazard_map_size_x_neg)*(hazard_map_size_y*2));
	int index = 0;

	

	//hazard_map_cells is a vector of vectors, each element of it is a grid in the hazard map that includes 0-N points
	for (int i = 0; i< hazard_filtered->points.size(); i++)
	{
	    point.push_back(hazard_filtered->points[i].x);
	    point.push_back(hazard_filtered->points[i].y);
	    point.push_back(hazard_filtered->points[i].z);

	    //the index checks which gird a point belongs to 

	    //**********************************
	    //maybe this is right, need to check
	    //**********************************

	    index = floor(hazard_filtered->points[i].x + hazard_map_size_x_neg) * (2 * hazard_map_size_y) + floor(hazard_filtered->points[i].y + hazard_map_size_y);

	    hazard_map_cells[index].push_back(point);
	    point.clear();
	}

	//do the calculation
	_hazard_x.clear();
	_hazard_y.clear();
	for (int i = 0; i < hazard_map_cells.size(); i++) // for every cell
	{
		//cout << i << endl;
		//define variables used to calculate mean x y z and variance of z
		float total_x = 0;
		float total_y = 0;
		float total_z = 0;
		float average_x = 0;
		float average_y = 0;
		float average_z = 0;
		float variance_z = 0;

	    for (int j = 0; j < hazard_map_cells[i].size(); j++)
	    {
	        total_x += hazard_map_cells[i][j][0];
	        total_y += hazard_map_cells[i][j][1];
	        total_z += hazard_map_cells[i][j][2];
	    }
	    average_x = total_x/hazard_map_cells[i].size();
	    average_y = total_y/hazard_map_cells[i].size();
	    average_z = total_z/hazard_map_cells[i].size();
	    for (int j = 0; j < hazard_map_cells[i].size(); j++)
	    {
	        variance_z = (hazard_map_cells[i][j][2]-average_z) * (hazard_map_cells[i][j][2]-average_z);
	    }
	    variance_z = sqrt(variance_z);

	    //the point should have at least one of the x, y or z not equal to 0 inorder to be included in the local map

	    //**********************************
	    //the threshold of variance_z can be adjusted as well
	    //**********************************

	    // if ((total_x || total_y || total_z) && variance_z > 0.3) //this is strange, what is this supposed to do?
	    if (total_x || total_y || total_z)
	    {
	 
	        _hazard_x.push_back(average_x);
			_hazard_y.push_back(average_y);
		  
	    }
	}

	ROS_INFO_STREAM("save PCD.........");
	pcl::PointCloud<pcl::PointXYZ>::Ptr testPCD (new pcl::PointCloud<pcl::PointXYZ>);
	for(int i = 0; i < _hazard_x.size(); i++)
	{
		testPCD->push_back(pcl::PointXYZ(_hazard_x[i], _hazard_y[i], 0));
	}

	pcl::io::savePCDFileASCII ("testPCD.pcd", *testPCD);


}

void CollisionDetection::generateHazardmap()
{

	//trigger the hazard map generation function
	pcl::PointCloud<pcl::PointXYZI>::Ptr hazard_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    *hazard_cloud = _input_cloud;

    int hazard_map_size = 30; //so each side is 30*2 = 60 

	//remove points based on hard thresholds (too far, too high, too low)
	pcl::PassThrough<pcl::PointXYZI> pass;
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

	//******************************
	//the general idea of this part is similar as local map generation, the difference is that the RANSAC fitting is more loose which will put 
	//more points to the ground, therefore the object cluster (hazard cluster) only has real hazarad, therefore, the false alarm rate will reduce
	//In other words, hazard map and path planning only consider real big hazard while small obstacle should be leave to the pure reactive layer
	//******************************

	plane.setDistanceThreshold (0.75); //ground detection threshold parameter
	plane.setInputCloud (hazard_cloud); //was raw_cloud

	//segment the points fitted to the plane using ransac
	pcl::SACSegmentation<pcl::PointXYZI> seg_plane;
	seg_plane.setOptimizeCoefficients (true); //optional (why is this optional??)
	seg_plane.setModelType (pcl::SACMODEL_PLANE);
	seg_plane.setMethodType (pcl::SAC_RANSAC);
	seg_plane.setMaxIterations (1000); //max iterations for RANSAC
	seg_plane.setDistanceThreshold (0.75); //ground detection threshold parameter
	seg_plane.setInputCloud (hazard_cloud); //was raw_cloud
	pcl::ModelCoefficients::Ptr coefficients_hazard (new pcl::ModelCoefficients ()); 
	pcl::PointIndices::Ptr inliers_hazard (new pcl::PointIndices ()); 
	seg_plane.segment (*inliers_hazard, *coefficients_hazard);
	ROS_INFO_STREAM("test0......................");
	//seperate the ground points and the points above the ground (object points)
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud (hazard_cloud);
	extract.setIndices (inliers_hazard);

	extract.setNegative (true);
	pcl::PointCloud<pcl::PointXYZI>::Ptr hazard_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	extract.filter (*hazard_filtered);

    //define variables used in this section
	std::vector<float> point;
	std::vector<std::vector<std::vector<float> > > hazard_map_cells((hazard_map_size*2)*(hazard_map_size*2));
	int index = 0;
	ROS_INFO_STREAM("test1.....................");
	//hazard_map_cells is a vector of vectors, each element of it is a grid in the hazard map that includes 0-N points
	for (int i = 0; i< hazard_filtered->points.size(); i++)
	{
	    point.push_back(hazard_filtered->points[i].x);
	    point.push_back(hazard_filtered->points[i].y);
	    point.push_back(hazard_filtered->points[i].z);

	    //the index checks which gird a point belongs to 

	    //**********************************
	    //maybe this is right, need to check
	    //**********************************

	    index = floor(hazard_filtered->points[i].x + hazard_map_size)*(2*hazard_map_size) + floor(hazard_filtered->points[i].y + hazard_map_size);

	    hazard_map_cells[index].push_back(point);
	    point.clear();
	}
	ROS_INFO_STREAM("test2.....................");
	//do the calculation
	_hazard_map_x.clear();
	_hazard_map_y.clear();
	for (int i = 0; i < hazard_map_cells.size(); i++) // for every cell
	{
		//cout << i << endl;
		//define variables used to calculate mean x y z and variance of z
		float total_x = 0;
		float total_y = 0;
		float total_z = 0;
		float average_x = 0;
		float average_y = 0;
		float average_z = 0;
		float variance_z = 0;

	    for (int j = 0; j < hazard_map_cells[i].size(); j++)
	    {
	        total_x += hazard_map_cells[i][j][0];
	        total_y += hazard_map_cells[i][j][1];
	        total_z += hazard_map_cells[i][j][2];
	    }
	    average_x = total_x/hazard_map_cells[i].size();
	    average_y = total_y/hazard_map_cells[i].size();
	    average_z = total_z/hazard_map_cells[i].size();
	    for (int j = 0; j < hazard_map_cells[i].size(); j++)
	    {
	        variance_z = (hazard_map_cells[i][j][2]-average_z) * (hazard_map_cells[i][j][2]-average_z);
	    }
	    variance_z = sqrt(variance_z);

	    //the point should have at least one of the x, y or z not equal to 0 inorder to be included in the local map

	    //**********************************
	    //the threshold of variance_z can be adjusted as well
	    //**********************************

	    // if ((total_x || total_y || total_z) && variance_z > 0.3) //this is strange, what is this supposed to do?
	    if (total_x || total_y || total_z)
	    {
	    	_hazard_map_x.push_back(average_x);
			_hazard_map_y.push_back(average_y);
	    }
	}
	// ROS_INFO_STREAM("save PCD2.........");
	// pcl::PointCloud<pcl::PointXYZ>::Ptr testPCD (new pcl::PointCloud<pcl::PointXYZ>);
	// for(int i = 0; i < _hazard_map_x.size(); i++)
	// {
	// 	testPCD->push_back(pcl::PointXYZ(_hazard_map_x[i], _hazard_map_y[i], 0));
	// }

	// pcl::io::savePCDFileASCII ("testPCD1.pcd", *testPCD);
}

bool CollisionDetection::returnHazardMap(messages::CreateROIHazardMap::Request &req, messages::CreateROIHazardMap::Response &res)
{
	res.x_mean.clear();
	res.y_mean.clear();

	generateHazardmap();

	for(int i=0; i<_hazard_map_x.size();i++)
	{
		res.x_mean.push_back(_hazard_map_x[i]);
		res.y_mean.push_back(_hazard_map_y[i]);
	}
	return true;
}

int CollisionDetection::firstChoice(double angle, double distance)
{
	int count = 0;
	for(int i = 0; i < _hazard_x.size(); i++)
	{
		if(_hazard_x[i] > - 1 / tan(angle) * _hazard_y[i] && _hazard_x[i] < - 1 / tan(angle) * _hazard_y[i] + distance / sin(angle))
		{
			if(fabs(cos(angle) * (_hazard_x[i] - _hazard_y[i] * tan(angle))) < threshold_obstacle_distance)
			{
				count++;
			}
		}			
	}

	return count;
}

int CollisionDetection::secondChoice(double angle, double distance, double xg, double yg)
{
	int count = 0;
	if(xg > distance * cos(angle))	//waypoint on top of temp point
	{
		for(int i = 0; i < _hazard_x.size(); i++)
		{
			if(_hazard_x[i] > distance * cos(angle))
			{
				if(fabs(_hazard_x[i] - ((distance * sin(angle) - xg) / (distance * cos(angle) - yg)) * _hazard_y[i] - (xg * distance * cos(angle) - yg * distance * sin(angle)) / (distance * cos(angle) - yg)) /
				sqrt(1 + pow((distance * sin(angle) - xg) / (distance * cos(angle) - yg),2)) < threshold_obstacle_distance)
				{
					count++;
				}
			}
		}
		
	}
	else
	{
		for(int i = 0; i < _hazard_x.size(); i++)
		{
			if(_hazard_x[i] < distance * cos(angle))
			{
				if(fabs(_hazard_x[i] - ((distance * sin(angle) - xg) / (distance * cos(angle) - yg)) * _hazard_y[i] - (xg * distance * cos(angle) - yg * distance * sin(angle)) / (distance * cos(angle) - yg)) /
				sqrt(1 + pow((distance * sin(angle) - xg) / (distance * cos(angle) - yg),2)) < threshold_obstacle_distance)
				{
					count++;
				}
			}
		}
		
	}

	ROS_INFO_STREAM("count: " <<count);

	return count;
}
