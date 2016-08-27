#include <computer_vision/sample_search.hpp>

SampleSearch::SampleSearch()
{
	searchForSamplesServ = nh.advertiseService("/vision/samplesearch/searchforsamples", &SampleSearch::searchForSamples, this);
	setSetSampleParamsServ = nh.advertiseService("/vision/samplesearch/setsampleparams", &SampleSearch::setSampleParams, this);
	segmentImageClient = nh.serviceClient<computer_vision::SegmentImage>("/vision/segmentation/segmentimage");
	classifierClient = nh.serviceClient<computer_vision::ImageProbabilities>("/classify_feature_vector_service");
	extractColorClient = nh.serviceClient<computer_vision::ExtractColor>("/vision/segmentation/extractcolor");
	searchForSamplesPub = nh.advertise<messages::CVSamplesFound>("vision/samplesearch/samplesearchout",1);
	
	// //initialize roi information
	boost::filesystem::path P( ros::package::getPath("computer_vision") );
	std::string roi_path = P.string() + "/data/roi_images/";
	for(int i=0; i<MAX_NUMBER_OF_ROIS; i++)
	{
		roi_t roi;
		roi.probabilities.clear();
		roi.probabilities.push_back(0);
		roi.probabilities.push_back(0);
		roi.probabilities.push_back(0);
		roi.types.clear();
		roi.types.push_back(_unknown_t);
		roi.types.push_back(_unknown_t);
		roi.types.push_back(_unknown_t);
		roi.paths.clear(); 
		roi.paths.push_back(roi_path + "roi" + patch::to_string(i) + "_" + "blob0.jpg");
		roi.paths.push_back(roi_path + "roi" + patch::to_string(i) + "_" + "blob1.jpg");
		roi.paths.push_back(roi_path + "roi" + patch::to_string(i) + "_" + "blob2.jpg");
		_rois.push_back(roi);	
	}

	// //initialize coefficients for mapping deep fish net probabilities to actual probabilities based on statistics
	// cachSigCoef.clear();
	// cachSigCoef.push_back(0.014343043024976);
	// cachSigCoef.push_back(0.996308298545352);
	// cachSigCoef.push_back(0.556984792625824);
	// cachSigCoef.push_back(12.655992533439981)
	// hardSigCoef.clear();
	// hardSigCoef.push_back(0.009590048374287);
	// hardSigCoef.push_back(0.996948659156515);
	// hardSigCoef.push_back(0.513078429713242);
	// hardSigCoef.push_back(15.391445601972140);
	// rockSigCoef.clear();
	// rockSigCoef.push_back(0.046567867798548);
	// rockSigCoef.push_back(0.996253031121210);
	// rockSigCoef.push_back(0.472406154111716);
	// rockSigCoef.push_back(21.523462524555410);
}

void SampleSearch::createFolderForImageData()
{
    boost::filesystem::path P( ros::package::getPath("computer_vision") );
    G_data_folder_name = patch::currentDateTime();
    G_data_folder = boost::filesystem::path(P.string()+"/data/saved/" + G_data_folder_name);
    boost::filesystem::create_directory(G_data_folder);
    G_data_folder_full_images = boost::filesystem::path(G_data_folder.string()+"/full");
    boost::filesystem::create_directory(G_data_folder_full_images);
    G_image_index = 0;
}

void SampleSearch::createFileForImageData()
{
	G_info_filename = G_data_folder.string() + "/" + patch::currentDateTime() + "_info.txt";
    G_outputInfoFile.open(G_info_filename.c_str(), std::ofstream::out | std::ofstream::trunc);
    G_outputInfoFile.close();
}

void SampleSearch::loadCalibrationData()
{
	bool error = false;

    //get path to calibration parameters
    boost::filesystem::path P( ros::package::getPath("computer_vision") );
    std::string filename = P.string() + "/data/calibration/cameraCalibration.csv";

    //open file containing calibration parameters
    std::ifstream inputFile;
    inputFile.open(filename.c_str());
    if(!inputFile)
    {
        ROS_ERROR("Error! Failed to open calibration file.");
        error = true;
    }

    //load roll, pitch, and yaw offsets of camera (wrt to the camera frame of reference)
    //note that x is the optical axis, y is positive to the right of the camera, and z is positive down
    if(inputFile.eof()) error=true;
    double roll;
	inputFile >> roll;
	roll = -roll; //need the reverse direction
	if(inputFile.eof()) error=true;
	double pitch;
    inputFile >> pitch;
    pitch = -pitch; //need the reverse direction
    if(inputFile.eof()) error=true;
    double yaw;
    inputFile >> yaw;
    yaw = -yaw; //need the reverse direction

    //close file
    inputFile.close();

    if(error==true)
    {
    	roll = 0;
    	pitch = 0;
    	yaw = 0;
    	ROS_WARN("Could not load calibration parameters for camera. Using identity for camera attitude offset...");
    }

    //rotation from uncalibrated camera to calibrated camera 
    cv::Mat R1x = (cv::Mat_<double>(3,3) << 1,       0,           0,     
    										0,       cos(roll),  -sin(roll),     
    										0,       sin(roll),   cos(roll));
    cv::Mat R1y = (cv::Mat_<double>(3,3) << cos(pitch),  0,       sin(pitch),    
                                            0,           1,       0,     
                                            -sin(pitch), 0,       cos(pitch));
    cv::Mat R1z = (cv::Mat_<double>(3,3) << cos(yaw),   -sin(yaw),    0,     
    	                                    sin(yaw),    cos(yaw),    0,     
    	                                    0,           0,           1);
    cv::Mat rotation_uncalibrated_to_calibrated_camera = R1x*R1y*R1z;

    //rotation from calibrated camera to robot body frame
    cv::Mat R2x = (cv::Mat_<double>(3,3) << 1,    0,         0,     
    										0,    cos(0),   -sin(0),     
    										0,    sin(0),    cos(0));
    cv::Mat R2y = (cv::Mat_<double>(3,3) << cos(-3.14159265/2),  0,  sin(-3.14159265/2),    
                                            0,                   1,  0,     
                                            -sin(-3.14159265/2), 0,  cos(-3.14159265/2));
    cv::Mat R2z = (cv::Mat_<double>(3,3) << cos(0),   -sin(0),    0,     
    	                                    sin(0),    cos(0),    0,     
    	                                    0,         0,         1);
    cv::Mat calibrated_camera_to_robot = R2x*R2y*R2z;

    //rotation from uncalibrated camera to robot body frame
    G_rotation_camera_2_robot = calibrated_camera_to_robot*rotation_uncalibrated_to_calibrated_camera;
}

std::vector<double> SampleSearch::calculateFlatGroundPositionOfPixel(int u, int v)
{
	//NOTE THAT 
	//X AXIS IS THE OPTICAL AXIS (POSITIVE OUT OF LENS), 
	//Y AXIS IS POSITIVE TO THE RIGHT WRT THE CAMERA,
	//Z AXIS IS POSITIVE DOWN WRT THE CAMERA

	//transform pixels to coordinate system centered around the image center
	u = u - G_IMAGE_WIDTH/2;
	v = v - G_IMAGE_HEIGHT/2;

	//transform u and v pixels to align with camera z and y 
	double zi = v; //pixels
	double yi = u; //pixels

	//calculate distance from center of image sensor
	float pixel_distance_from_center = sqrt(zi*zi + yi*yi); //mm

	//convert distance to angle of view (source http://wiki.panotools.org/Fisheye_Projection)
	//this is the angle between the optical axis and the sample position 
	double angle_between_optical_and_sample = pixel_distance_from_center*G_SIZE_OF_PIXEL/G_FOCAL_LENGTH; //radians

	//calculate distance assuming flat ground tan(angle) = (horizontal distance to sample)/(vertical distance to sample)
	double flat_ground_distance_distance = G_SENSOR_HEIGHT*tan(angle_between_optical_and_sample);

	//calculate angle about the x axis to the sample (angle to rotate to face the sample)
	double delta_angle = atan2(yi,zi);

	//convert to cartesian coordinates
	double x = 0;
	double y = flat_ground_distance_distance*sin(delta_angle);
	double z = flat_ground_distance_distance*cos(delta_angle);
	cv::Mat sample_position = (cv::Mat_<double>(3,1) << x, y, z);

	//apply rotation from calibration
    cv::Mat sample_position_t = G_rotation_camera_2_robot*sample_position;
    double x_t = sample_position_t.at<double>(0,0) + 0.45;
    double y_t = sample_position_t.at<double>(1,0);
    double z_t = sample_position_t.at<double>(2,0);

    //convert back to polar for output
	double flat_ground_distance_distance_t = sqrt(x_t*x_t + y_t*y_t);
	double delta_angle_t = atan2(y_t,x_t);

	// ROS_INFO("x, y, z = %f, %f, %f", x, y, z);
	// ROS_INFO("d, bearing (before calibration) = %f, %f", flat_ground_distance_distance, atan2(yi,-zi)*180/3.14159265);
	// ROS_INFO("x_t, y_t, z_t = %f, %f, %f", x_t, y_t, z_t);
	// ROS_INFO("d_t, bearing_t = %f, %f", flat_ground_distance_distance_t, delta_angle_t*180/3.14159265);

	//push back output in vector
	std::vector<double> relative_position;
	if(angle_between_optical_and_sample > 3.1415926/2)
	{
		ROS_ERROR("Error! Object detectable outside of visible region of image...");
		ROS_ERROR("flat_ground_distance_distance_t = %f", flat_ground_distance_distance_t);
		ROS_ERROR("delta_angle_t*180/3.14159265 = %f", delta_angle_t*180/3.14159265);
		ROS_ERROR("angle_between_optical_and_sample*180/3.14159265 = %f", angle_between_optical_and_sample*180/3.14159265);
		relative_position.push_back(0);
		relative_position.push_back(0);
	}
	else
	{
		relative_position.push_back(flat_ground_distance_distance_t);
		relative_position.push_back(delta_angle_t*180/3.14159265);
	}

	//return the relative position with flat ground assumption
	return relative_position;
}

void SampleSearch::drawResultsOnImage(const std::vector<int> &blobsOfInterest, const std::vector<int> &blobsOfNotInterest, const std::vector<int> &coordinates, const std::vector<int> &types, const std::vector<float> &probabilities)
{
	//load image
	boost::filesystem::path P( ros::package::getPath("computer_vision") );
	cv::Mat src = cv::imread(P.string()+"/data/images/input_image.jpg");

	//draw circle for detected samples
	for(int i=0; i<blobsOfInterest.size(); i++)
	{
		std::vector<int> color = map_enum_to_color(static_cast<SAMPLE_TYPE_T>(types[i]));
		circle(src, cv::Point(coordinates[blobsOfInterest[i]*2],coordinates[blobsOfInterest[i]*2+1]), 100, cv::Scalar(color[0], color[1], color[2]), 3, 8);
		circle(src, cv::Point(coordinates[blobsOfInterest[i]*2],coordinates[blobsOfInterest[i]*2+1]), 5, cv::Scalar(0,0,0), 3, 8);

		float scale_img  = 600.f/(sqrt(5792*5792*2.5));
		float scale_font = (float)(2-scale_img)/1.4f;

		int tempProb = 100*probabilities[blobsOfInterest[i]];
		cv::Size word_size = getTextSize(patch::to_string(tempProb), cv::FONT_HERSHEY_SIMPLEX, (double)scale_font, (int)(3*scale_font), NULL);
		rectangle(src, cv::Point(coordinates[blobsOfInterest[i]*2], coordinates[blobsOfInterest[i]*2+1] + 100)-cv::Point(3,word_size.height+3), cv::Point(coordinates[blobsOfInterest[i]*2], coordinates[blobsOfInterest[i]*2+1] + 100)+cv::Point(word_size.width,0), cv::Scalar(0,0,0),-1);
		cv::putText(src, patch::to_string(tempProb), cv::Point(coordinates[blobsOfInterest[i]*2], coordinates[blobsOfInterest[i]*2+1] + 100)-cv::Point(1,1), cv::FONT_HERSHEY_SIMPLEX, scale_font, cv::Scalar(255,255,255),(int)(3*scale_font));
	}

	//draw circles for detected nonsample
	for(int i=0; i<blobsOfNotInterest.size(); i++)
	{
		circle(src, cv::Point(coordinates[blobsOfNotInterest[i]*2],coordinates[blobsOfNotInterest[i]*2+1]), 100, cv::Scalar(0,255,0), 3, 8);
		circle(src, cv::Point(coordinates[blobsOfNotInterest[i]*2],coordinates[blobsOfNotInterest[i]*2+1]), 5, cv::Scalar(0,0,0), 3, 8);
		float scale_img  = 600.f/(sqrt(5792*5792*2.5));
		float scale_font = (float)(2-scale_img)/1.4f;
		int tempProb = 100*probabilities[blobsOfNotInterest[i]];
		cv::Size word_size = getTextSize(patch::to_string(tempProb), cv::FONT_HERSHEY_SIMPLEX, (double)scale_font, (int)(3*scale_font), NULL);
		rectangle(src, cv::Point(coordinates[blobsOfNotInterest[i]*2], coordinates[blobsOfNotInterest[i]*2+1] + 100)-cv::Point(3,word_size.height+3), cv::Point(coordinates[blobsOfNotInterest[i]*2], coordinates[blobsOfNotInterest[i]*2+1] + 100)+cv::Point(word_size.width,0), cv::Scalar(0,0,0),-1);
		cv::putText(src, patch::to_string(tempProb), cv::Point(coordinates[blobsOfNotInterest[i]*2], coordinates[blobsOfNotInterest[i]*2+1] + 100)-cv::Point(1,1), cv::FONT_HERSHEY_SIMPLEX, scale_font, cv::Scalar(255,255,255),(int)(3*scale_font));
	}


	//write image to file
	imwrite(P.string() + "/data/images/output_image.jpg", src);
}

//NOTE: This function may not write the blob info to the file correctly... Needs to be verified...
void SampleSearch::saveLowAndHighProbabilityBlobs(const std::vector<float> &probabilities, const std::vector<int> &coordinates)
{
	boost::filesystem::path P( ros::package::getPath("computer_vision") );
	int save_image_flag = 0;
	int copy_original_cols = 0;
	int copy_original_rows = 0;
	for(int i=0; i<probabilities.size(); i++)
	{
		if(probabilities[i]>0.2) //save blobs with probability above 0.2
		{
		    //write full image to file
			if(save_image_flag==0) //only save full image once
			{
				save_image_flag=1;
				G_blob_image_name = patch::currentDateTime() + ".jpg";
				boost::filesystem::path copy_from_full(P.string() + "/data/images/input_image.jpg");
				boost::filesystem::path copy_to_full( G_data_folder_full_images.string() + "/" + G_blob_image_name);
				cv::Mat image_copy_from_full = cv::imread(copy_from_full.string());
				copy_original_cols = image_copy_from_full.cols;
				copy_original_rows = image_copy_from_full.rows;
				cv::imwrite(copy_to_full.string(),image_copy_from_full);
			}

			//write blob to file
			boost::filesystem::path copy_from_blob(P.string() + "/data/blobs/blob" + patch::to_string(i) + ".jpg");
			boost::filesystem::path copy_to_blob( G_data_folder.string() + "/img" + patch::to_string(G_image_index) + "_blob" + patch::to_string(i) + "_folder" + G_data_folder_name + ".jpg");
			cv::Mat image_copy_from_blob = cv::imread(copy_from_blob.string());
			cv::imwrite(copy_to_blob.string(),image_copy_from_blob);

			//write blob info to file
		    G_outputInfoFile.open(G_info_filename.c_str()); //open the file
            G_outputInfoFile << "img" + patch::to_string(G_image_index) + "_blob" + patch::to_string(i) + "_" + G_data_folder_name + ".jpg"; //name of blob image
            G_outputInfoFile << "," << G_data_folder_name + "/full/" + G_blob_image_name; //folder containing full image
            G_outputInfoFile << "," << copy_original_cols; //width of origional image
            G_outputInfoFile << "," << copy_original_rows; //height of origional image
            G_outputInfoFile << "," << coordinates[i*2]; //center of object x
            G_outputInfoFile << "," << coordinates[i*2+1]; //center of obejct y
            G_outputInfoFile << "," << image_copy_from_blob.cols*0.5; //origional width of segment, NOTE: this needs to be changed if segmentation class changes
            G_outputInfoFile << "," << image_copy_from_blob.rows*0.5; //origional height of segment, NOTE: this needs to be changed if segmentation class changes
            G_outputInfoFile << "," << image_copy_from_blob.cols; //expanded width of segment
            G_outputInfoFile << "," << image_copy_from_blob.rows; //expanded height of segment
            G_outputInfoFile << std::endl; //end line for each blob
		    G_outputInfoFile.close(); //close the file (must include this in the case the program crashes)
		}
	}
	G_image_index++;
}

void SampleSearch::saveTopROICandidates(const std::vector<int> &blobsOfInterest,
										const std::vector<int> &blobsOfNotInterest, 
										const std::vector<int> &types, 
										const std::vector<float> &probabilities, 
										const int &roi)
{
	//filepath for computer vision package
	boost::filesystem::path P( ros::package::getPath("computer_vision") );
	// ROS_INFO("%i,%i,%i,%i,%i = ", blobsOfInterest.size(), blobsOfNotInterest.size(), types.size(), probabilities.size(), roi);

	if(roi < MAX_NUMBER_OF_ROIS && roi >= 0)
	{
		for(int i=0; i<blobsOfInterest.size(); i++)
		{
			// ROS_INFO("PART1 probC, prob0, prob1, prob2 = %f, %f, %f, %f", probabilities[blobsOfInterest[i]],_rois[roi].probabilities[0],_rois[roi].probabilities[1],_rois[roi].probabilities[2]);
			if(probabilities[blobsOfInterest[i]] > _rois[roi].probabilities[0])
			{
				// ROS_INFO(">0");
				_rois[roi].probabilities[2] = _rois[roi].probabilities[1];
				_rois[roi].probabilities[1] = _rois[roi].probabilities[0];
				_rois[roi].probabilities[0] = probabilities[blobsOfInterest[i]];

				_rois[roi].types[2] = _rois[roi].types[1];
				_rois[roi].types[1] = _rois[roi].types[0];
				_rois[roi].types[0] = static_cast<SAMPLE_TYPE_T>(types[i]);

				cv::imwrite( _rois[roi].paths[2], cv::imread(_rois[roi].paths[1]) );
				cv::imwrite( _rois[roi].paths[1], cv::imread(_rois[roi].paths[0]) );
				cv::imwrite( _rois[roi].paths[0], cv::imread(P.string() + "/data/blobs/blob" + patch::to_string(blobsOfInterest[i]) + ".jpg") );
			}
			else if(probabilities[blobsOfInterest[i]] > _rois[roi].probabilities[1])
			{
				// ROS_INFO(">1");
				_rois[roi].probabilities[2] = _rois[roi].probabilities[1];
				_rois[roi].probabilities[1] = probabilities[blobsOfInterest[i]];

				_rois[roi].types[2] = _rois[roi].types[1];
				_rois[roi].types[1] = static_cast<SAMPLE_TYPE_T>(types[i]);

				cv::imwrite( _rois[roi].paths[2], cv::imread(_rois[roi].paths[1]) );
				cv::imwrite( _rois[roi].paths[1], cv::imread(P.string() + "/data/blobs/blob" + patch::to_string(blobsOfInterest[i]) + ".jpg") );
			}
			else if (probabilities[blobsOfInterest[i]] > _rois[roi].probabilities[2])
			{
				// ROS_INFO(">2");
				_rois[roi].probabilities[2] = probabilities[blobsOfInterest[i]];
				_rois[roi].types[2] = static_cast<SAMPLE_TYPE_T>(types[i]);
				cv::imwrite( _rois[roi].paths[2], cv::imread(P.string() + "/data/blobs/blob" + patch::to_string(blobsOfInterest[i]) + ".jpg") );
			}
		}

		for(int i=0; i<blobsOfNotInterest.size(); i++)
		{
			// ROS_INFO("PART2 probC, prob0, prob1, prob2 = %f, %f, %f, %f", probabilities[blobsOfInterest[i]], _rois[roi].probabilities[0], _rois[roi].probabilities[1], _rois[roi].probabilities[2]);
			if(probabilities[blobsOfNotInterest[i]] > _rois[roi].probabilities[0])
			{
				// ROS_INFO(">0");
				_rois[roi].probabilities[2] = _rois[roi].probabilities[1];
				_rois[roi].probabilities[1] = _rois[roi].probabilities[0];
				_rois[roi].probabilities[0] = probabilities[blobsOfNotInterest[i]];

				_rois[roi].types[2] = _rois[roi].types[1];
				_rois[roi].types[1] = _rois[roi].types[0];
				_rois[roi].types[0] = _unknown_t;

				cv::imwrite( _rois[roi].paths[2], cv::imread(_rois[roi].paths[1]) );
				cv::imwrite( _rois[roi].paths[1], cv::imread(_rois[roi].paths[0]) );
				cv::imwrite( _rois[roi].paths[0], cv::imread(P.string() + "/data/blobs/blob" + patch::to_string(blobsOfNotInterest[i]) + ".jpg") );
			}
			else if(probabilities[blobsOfNotInterest[i]] > _rois[roi].probabilities[1])
			{
				// ROS_INFO(">1");
				_rois[roi].probabilities[2] = _rois[roi].probabilities[1];
				_rois[roi].probabilities[1] = probabilities[blobsOfNotInterest[i]];

				_rois[roi].types[2] = _rois[roi].types[1];
				_rois[roi].types[1] = _unknown_t;

				cv::imwrite( _rois[roi].paths[2], cv::imread(_rois[roi].paths[1]) );
				cv::imwrite( _rois[roi].paths[1], cv::imread(P.string() + "/data/blobs/blob" + patch::to_string(blobsOfNotInterest[i]) + ".jpg") );
			}
			else if (probabilities[blobsOfNotInterest[i]] > _rois[roi].probabilities[2])
			{
				// ROS_INFO(">2");
				_rois[roi].probabilities[2] = probabilities[blobsOfNotInterest[i]];
				_rois[roi].types[2] = _unknown_t;
				cv::imwrite( _rois[roi].paths[2], cv::imread(P.string() + "/data/blobs/blob" + patch::to_string(blobsOfNotInterest[i]) + ".jpg") );
			}
		}
	}
}

bool SampleSearch::searchForSamples(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res)
{
	gettimeofday(&this->localTimer, NULL);
	double startSearchTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  

	/*
		Call segmentation server
	*/
	gettimeofday(&this->localTimer, NULL);
    double startSegmentationTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  

	segmentImageSrv.request.live = req.live;
	segmentImageSrv.request.path = req.path;
	if(segmentImageClient.call(segmentImageSrv))
	{
		ROS_INFO("segmentImageSrv call successful!");
	}
	else
	{
		ROS_ERROR("Error! Failed to call service SegmentImage!");
		searchForSamplesMsgOut.sampleList.clear();
		searchForSamplesMsgOut.procType = req.procType;
		searchForSamplesMsgOut.serialNum = req.serialNum;
		searchForSamplesPub.publish(searchForSamplesMsgOut);
		ros::spinOnce();
		return true;
	}

	gettimeofday(&this->localTimer, NULL);  
    double endSegmentationTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for segmentation service: %f", endSegmentationTime - startSegmentationTime);

    //do not call classifier if no blobs or too many  blobs were extracted from segmentation
	if(segmentImageSrv.response.coordinates.size()/2 < 1 && segmentImageSrv.response.coordinates.size()/2 > 1000)
	{
		ROS_WARN("No blobs detected in image. Not performing classification.");
		searchForSamplesMsgOut.sampleList.clear();
		searchForSamplesMsgOut.procType = req.procType;
		searchForSamplesMsgOut.serialNum = req.serialNum;
		searchForSamplesPub.publish(searchForSamplesMsgOut);
		ros::spinOnce();
		return true;
	}

	/*
		Call classifier server
	*/
	gettimeofday(&this->localTimer, NULL);
    double startClassifierTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  

    //ALL SAMPLE CLASSIFIER (THIS WILL BE REMOVED ONCE THE OTHER CLASSIFIERS ARE TRAINED)
	imageProbabilitiesSrv.request.numBlobs = segmentImageSrv.response.coordinates.size()/2;
	imageProbabilitiesSrv.request.imgSize = 50; //50 will do 50x50 classifier, 150 will do 150x150 classifier (BUT 150x150 no longer exists)
	imageProbabilitiesSrv.request.classifierType = 2;
	if(classifierClient.call(imageProbabilitiesSrv))
	{
		ROS_INFO("imageProbabilitiesSrv call successful!");
	}
	else
	{
		ROS_ERROR("Error! Failed to call service ImageProbabilities!");
		searchForSamplesMsgOut.sampleList.clear();
		searchForSamplesMsgOut.procType = req.procType;
		searchForSamplesMsgOut.serialNum = req.serialNum;
		searchForSamplesPub.publish(searchForSamplesMsgOut);
		ros::spinOnce();
		return true;
	}

	G_cach_probabilities.clear();
	G_hard_probabilities.clear();
	G_rock_probabilities.clear();
	G_sample_probabilities.clear();
	G_sample_ids.clear();
	if(req.roi == 0)
	{
		//precached sample classifier
		imageProbabilitiesSrv.request.numBlobs = segmentImageSrv.response.coordinates.size()/2;
		imageProbabilitiesSrv.request.imgSize = 50;
		imageProbabilitiesSrv.request.classifierType = 0;
		if(classifierClient.call(imageProbabilitiesSrv))
		{
			ROS_INFO("imageProbabilitiesSrv call successful with type 0 (cach)!");
			G_cach_probabilities = imageProbabilitiesSrv.response.responseProbabilities;
		}
		else
		{
			ROS_ERROR("Error! Failed to call service ImageProbabilities!");
			searchForSamplesMsgOut.sampleList.clear();
			searchForSamplesMsgOut.procType = req.procType;
			searchForSamplesMsgOut.serialNum = req.serialNum;
			searchForSamplesPub.publish(searchForSamplesMsgOut);
			ros::spinOnce();
			return true;
		}

		//assign probabilties and sample ids
		for(int i=0; i<G_cach_probabilities.size(); i++) //this is the number of blobs from segmentation
		{
			G_hard_probabilities.push_back(0);
			G_rock_probabilities.push_back(0);
			float tempMappedProbabilityCach = cachSigCoef[0]+(cachSigCoef[1]-cachSigCoef[0])/(1+pow(10,( (cachSigCoef[2]-G_cach_probabilities[i])*cachSigCoef[3]) ) );
			G_sample_probabilities.push_back(tempMappedProbabilityCach);
			G_sample_ids.push_back(0);
		}
	}
	else
	{
		//hard sample classifier
		imageProbabilitiesSrv.request.numBlobs = segmentImageSrv.response.coordinates.size()/2;
		imageProbabilitiesSrv.request.imgSize = 50;
		imageProbabilitiesSrv.request.classifierType = 1;
		if(classifierClient.call(imageProbabilitiesSrv))
		{
			ROS_INFO("imageProbabilitiesSrv call successful with type 1 (hard)!");
			G_hard_probabilities = imageProbabilitiesSrv.response.responseProbabilities;
		}
		else
		{
			ROS_ERROR("Error! Failed to call service ImageProbabilities!");
			searchForSamplesMsgOut.sampleList.clear();
			searchForSamplesMsgOut.procType = req.procType;
			searchForSamplesMsgOut.serialNum = req.serialNum;
			searchForSamplesPub.publish(searchForSamplesMsgOut);
			ros::spinOnce();
			return true;
		}

		//rock sample classifier
		imageProbabilitiesSrv.request.numBlobs = segmentImageSrv.response.coordinates.size()/2;
		imageProbabilitiesSrv.request.imgSize = 50;
		imageProbabilitiesSrv.request.classifierType = 2;
		if(classifierClient.call(imageProbabilitiesSrv))
		{
			ROS_INFO("imageProbabilitiesSrv call successful with type 2 (rock)!");
			G_rock_probabilities = imageProbabilitiesSrv.response.responseProbabilities;
		}
		else
		{
			ROS_ERROR("Error! Failed to call service ImageProbabilities!");
			searchForSamplesMsgOut.sampleList.clear();
			searchForSamplesMsgOut.procType = req.procType;
			searchForSamplesMsgOut.serialNum = req.serialNum;
			searchForSamplesPub.publish(searchForSamplesMsgOut);
			ros::spinOnce();
			return true;
		}

		//make sure the number of blobs is correct
		if(G_rock_probabilities.size() != G_hard_probabilities.size() || G_rock_probabilities.size() != segmentImageSrv.response.coordinates.size()/2)
		{
			ROS_ERROR("Error! The size of G_rock_probabilities and G_hard_probabilities should be equal to the number of blobs from segmentation!");
			G_hard_probabilities.clear();
			G_rock_probabilities.clear();
		}

		//assign probabilties and sample ids (use higher probability and the cooresponding id)
		G_sample_probabilities.clear();
		G_sample_ids.clear();
		for(int i=0; i<G_rock_probabilities.size(); i++) //this is the number of blobs in rock and hard probabilities
		{
			G_cach_probabilities.push_back(0);
			float tempMappedProbabilityHard = hardSigCoef[0]+(hardSigCoef[1]-hardSigCoef[0])/(1+pow(10,( (hardSigCoef[2]-G_hard_probabilities[i])*hardSigCoef[3]) ) );
			float tempMappedProbabilityRock = rockSigCoef[0]+(rockSigCoef[1]-rockSigCoef[0])/(1+pow(10,( (rockSigCoef[2]-G_rock_probabilities[i])*rockSigCoef[3]) ) );
			if(tempMappedProbabilityHard >= tempMappedProbabilityRock)
			{
				G_sample_probabilities.push_back(tempMappedProbabilityHard);
				G_sample_ids.push_back(1);
			}
			else
			{
				G_sample_probabilities.push_back(tempMappedProbabilityRock);
				G_sample_ids.push_back(0);
			}
		}
	}

	gettimeofday(&this->localTimer, NULL);  
    double endClassifierTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for classifier service: %f", endClassifierTime - startClassifierTime);

	/*
		Call color extract color service
	*/
	gettimeofday(&this->localTimer, NULL);
    double startColorTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0); 

    //find samples of interest for extracting colors (sample must have at least 0.1 confidence)
	std::vector<int> blobsOfInterest, blobsOfNotInterest;
	for(int i=0; i<G_sample_probabilities.size(); i++)
	{
		if(G_sample_probabilities[i]>0.1)
		{
			blobsOfInterest.push_back(i);
		}
		else
		{
			blobsOfNotInterest.push_back(i);
		}
	}	

	//call service to extract colors from samples (uses look up table)
	extractColorSrv.request.blobsOfInterest = blobsOfInterest;
	if(extractColorClient.call(extractColorSrv))
	{
		ROS_INFO("extractColorSrv call successful!");
	}
	else
	{
		ROS_ERROR("Error! Failed to call service ExtractColor!");
	}

	gettimeofday(&this->localTimer, NULL);  
    double endColorTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for extract color service: %f", endColorTime - startColorTime);

	/*
		Save image, blobs, and blob info if > 0.2 probability of being a sample
	*/
	gettimeofday(&this->localTimer, NULL);
    double startSaveBlobsTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  

	saveLowAndHighProbabilityBlobs(G_sample_probabilities, segmentImageSrv.response.coordinates);

	gettimeofday(&this->localTimer, NULL);  
    double endSaveBlobsTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for saving high probability blobs: %f", endSaveBlobsTime - startSaveBlobsTime);

	/*
		Draw samples and nonsamples on image (the should always be done, this should not be an option)
	*/
	gettimeofday(&this->localTimer, NULL);
    double startDrawTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  

	drawResultsOnImage(blobsOfInterest, blobsOfNotInterest, segmentImageSrv.response.coordinates, extractColorSrv.response.types, G_sample_probabilities);

	gettimeofday(&this->localTimer, NULL);  
    double endDrawTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for drawing output image: %f", endDrawTime - startDrawTime);

	/*
		Calculate the position and publish each sample requested
	*/
	gettimeofday(&this->localTimer, NULL);
    double startSampleLocalizationTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  

    std::vector<double> position;
	messages::CVSampleProps sampleProps;
	searchForSamplesMsgOut.sampleList.clear();
	for(int i=0; i<blobsOfInterest.size(); i++)
	{
		//position of sample
		position.clear();
		position = calculateFlatGroundPositionOfPixel(segmentImageSrv.response.coordinates[blobsOfInterest[i]*2], segmentImageSrv.response.coordinates[blobsOfInterest[i]*2+1]);
		sampleProps.distance = position[0];
		sampleProps.bearing = position[1];
		
		//type of sample
		sampleProps.white = false;
		sampleProps.silver = false;
		sampleProps.blueOrPurple = false;
		sampleProps.pink = false;
		sampleProps.red = false;
		sampleProps.orange = false;
		sampleProps.yellow = false;	
		std::vector<SAMPLE_TYPE_T> possibleTypes = map_to_simple_color( static_cast<SAMPLE_TYPE_T>(extractColorSrv.response.types[i]) );
		for(int j=0; j<possibleTypes.size(); j++) //change each type to true if possible
		{
			if(possibleTypes[j] == _white_t) sampleProps.white = true;
			if(possibleTypes[j] == _silver_t) sampleProps.silver = true;
			if(possibleTypes[j] == _blueOrPurple_t) sampleProps.blueOrPurple = true;
			if(possibleTypes[j] == _pink_t) sampleProps.pink = true;
			if(possibleTypes[j] == _red_t) sampleProps.red = true;
			if(possibleTypes[j] == _orange_t) sampleProps.orange = true;
			if(possibleTypes[j] == _yellow_t) sampleProps.yellow = true;
		}

		//confidence of sample
		sampleProps.confidenceCach = G_cach_probabilities[blobsOfInterest[i]];
		sampleProps.confidenceHard = G_hard_probabilities[blobsOfInterest[i]];
		sampleProps.confidenceRock = G_rock_probabilities[blobsOfInterest[i]];
		sampleProps.confidence = G_sample_probabilities[blobsOfInterest[i]];

		//only publish results depending on expected probabilities
		bool publish_sample_info = false;
		if(req.white > 0.6)
		{
			if(sampleProps.white == true) publish_sample_info = true;
		}
		else if(req.silver > 0.6)
		{
			if(sampleProps.silver == true) publish_sample_info = true;
		}
		else if(req.blueOrPurple > 0.6)
		{
			if(sampleProps.blueOrPurple == true) publish_sample_info = true;
		}
		else if(req.pink > 0.6 || req.red > 0.6)
		{
			if(sampleProps.red == true || sampleProps.pink == true) publish_sample_info = true;
		}
		else if(req.orange > 0.6)
		{
			if(sampleProps.orange == true) publish_sample_info = true;
		}
		else if(req.yellow > 0.6)
		{
			if(sampleProps.yellow == true) publish_sample_info = true;	
		}
		else
		{
			if(req.white > 0.03) if(sampleProps.white == true) publish_sample_info = true;
			if(req.silver > 0.03) if(sampleProps.silver == true) publish_sample_info = true;
			if(req.blueOrPurple > 0.03) if(sampleProps.blueOrPurple == true) publish_sample_info = true;
			if(req.red > 0.03 || req.pink > 0.03) if(sampleProps.red == true || sampleProps.pink == true) publish_sample_info = true;
			if(req.orange > 0.03) if(sampleProps.orange == true) publish_sample_info = true;
			if(req.yellow > 0.03) if(sampleProps.yellow == true) publish_sample_info = true;	
		}

		//add sample information to message
		if(publish_sample_info==true)
		{
			searchForSamplesMsgOut.sampleList.push_back(sampleProps);
		}
	}


	gettimeofday(&this->localTimer, NULL);  
    double endSampleLocalizationTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for sample localization: %f", endSampleLocalizationTime - startSampleLocalizationTime);

	/*
		Save top samples in ROI and set parameters with ROI info
	*/
	gettimeofday(&this->localTimer, NULL);
    double startROICandidatesTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  

	saveTopROICandidates(blobsOfInterest, blobsOfNotInterest, extractColorSrv.response.types, G_sample_probabilities, req.roi);

	gettimeofday(&this->localTimer, NULL);  
    double endROICandidatesTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for adding information to ROIs: %f", endROICandidatesTime - startROICandidatesTime);

	/*
		Publish results of search
	*/
    searchForSamplesMsgOut.procType = req.procType; //must return req.procType
    searchForSamplesMsgOut.serialNum = req.serialNum; //must return req.serialNum
    searchForSamplesPub.publish(searchForSamplesMsgOut);
    // ROS_INFO("searchForSamplesMsgOut.sampleList.size() = %i", searchForSamplesMsgOut.sampleList.size());

	gettimeofday(&this->localTimer, NULL);
	double endSearchTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);
	ROS_INFO("Total time for search: %f", endSearchTime - startSearchTime);

    ros::spinOnce(); //publish results before completing request (important!)
	return true;
}

bool SampleSearch::setSampleParams(messages::CVSetSampleParams::Request &req, messages::CVSetSampleParams::Response &res)
{
	if(req.roi < MAX_NUMBER_OF_ROIS)
	{
		std::string path1Param = "/vision/roi" + patch::to_string(req.roi) + "/path1";
		std::string path2Param = "/vision/roi" + patch::to_string(req.roi) + "/path2";
		std::string path3Param = "/vision/roi" + patch::to_string(req.roi) + "/path3";
		std::string conf1Param = "/vision/roi" + patch::to_string(req.roi) + "/confidence1";
		std::string conf2Param = "/vision/roi" + patch::to_string(req.roi) + "/confidence2";
		std::string conf3Param = "/vision/roi" + patch::to_string(req.roi) + "/confidence3";
		std::string type1Param = "/vision/roi" + patch::to_string(req.roi) + "/type1";
		std::string type2Param = "/vision/roi" + patch::to_string(req.roi) + "/type2";
		std::string type3Param = "/vision/roi" + patch::to_string(req.roi) + "/type3";

		nh.setParam(path1Param.c_str(), _rois[req.roi].paths[0]);
		nh.setParam(path2Param.c_str(), _rois[req.roi].paths[1]);
		nh.setParam(path3Param.c_str(), _rois[req.roi].paths[2]);
		nh.setParam(conf1Param.c_str(), _rois[req.roi].probabilities[0]);
		nh.setParam(conf2Param.c_str(), _rois[req.roi].probabilities[1]);
		nh.setParam(conf3Param.c_str(), _rois[req.roi].probabilities[2]);
		nh.setParam(type1Param.c_str(), _rois[req.roi].types[0]);
		nh.setParam(type2Param.c_str(), _rois[req.roi].types[1]);
		nh.setParam(type3Param.c_str(), _rois[req.roi].types[2]);
	}

	res.namespace_str = nh.getNamespace();
	return true;
}