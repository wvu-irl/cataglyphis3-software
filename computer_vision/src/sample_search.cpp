#include <computer_vision/sample_search.hpp>

SampleSearch::SampleSearch()
{
	searchForSamplesServ = nh.advertiseService("/vision/samplesearch/searchforsamples", &SampleSearch::searchForSamples, this);
	segmentImageClient = nh.serviceClient<computer_vision::SegmentImage>("/vision/segmentation/segmentimage");
	classifierClient = nh.serviceClient<computer_vision::ImageProbabilities>("/classify_feature_vector_service");
	searchForSamplesPub = nh.advertise<messages::CVSamplesFound>("vision/samplesearch/samplesearchout",1);
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

	//convert pixels to distance in mm (pixel is 4.14 micrometer square)
	// zi = zi*4.1436e-6; //mm
	// yi = yi*4.1436e-6; //mm

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
    double x_t = sample_position_t.at<double>(0,0) - 0.45;
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

void SampleSearch::drawResultsOnImage(const std::vector<int> &binary, const std::vector<int> &coordinates)
{
	//load image
	boost::filesystem::path P( ros::package::getPath("computer_vision") );
	cv::Mat src = cv::imread(P.string()+"/data/images/input_image.jpg");

	//draw circle for detected samples and nonsamples
	for(int i=0; i<binary.size(); i++)
	{
		if(binary[i]==1)
		{
			circle(src, cv::Point(coordinates[i*2],coordinates[i*2+1]), 100, cv::Scalar(0,0,255), 3, 8);
		}
		else
		{
			circle(src, cv::Point(coordinates[i*2],coordinates[i*2+1]), 100, cv::Scalar(0,0,0), 3, 8);
		}
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

    //do not call classifier if no blobs were extracted from segmentation
	if(segmentImageSrv.response.coordinates.size()/2 < 1)
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

	imageProbabilitiesSrv.request.numBlobs = segmentImageSrv.response.coordinates.size()/2;
	imageProbabilitiesSrv.request.imgSize = 50; //50 will do 50x50 classifier, 150 will do 150x150 classifier	
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

	gettimeofday(&this->localTimer, NULL);  
    double endClassifierTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for classifier service: %f", endClassifierTime - startClassifierTime);

	/*
		Save image, blobs, and blob info if > 0.2 probability of being a sample
	*/
	gettimeofday(&this->localTimer, NULL);
    double startSaveBlobsTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  

	saveLowAndHighProbabilityBlobs(imageProbabilitiesSrv.response.responseProbabilities, segmentImageSrv.response.coordinates);

	gettimeofday(&this->localTimer, NULL);  
    double endSaveBlobsTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for saving high probability blobs: %f", endSaveBlobsTime - startSaveBlobsTime);

	/*
		Draw samples and nonsamples on image (the should always be done, this should not be an option)
	*/
	gettimeofday(&this->localTimer, NULL);
    double startDrawTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  

	std::vector<int> binary;
	for(int i=0; i<imageProbabilitiesSrv.response.responseProbabilities.size(); i++)
	{
		if(imageProbabilitiesSrv.response.responseProbabilities[i]>0.50)
		{
			binary.push_back(1);
		}
		else
		{
			binary.push_back(0);
		}
	}
	drawResultsOnImage(binary, segmentImageSrv.response.coordinates);

	gettimeofday(&this->localTimer, NULL);  
    double endDrawTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for drawing output image: %f", endDrawTime - startDrawTime);

	/*
		Calculate the position of each sample
	*/
	gettimeofday(&this->localTimer, NULL);
    double startSampleLocalizationTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  

	std::vector<double> position;
	messages::CVSampleProps sampleProps;
	searchForSamplesMsgOut.sampleList.clear();
	for(int i=0; i<binary.size(); i++)
	{
		if(binary[i]==1)
		{
			position.clear();
			position = calculateFlatGroundPositionOfPixel(segmentImageSrv.response.coordinates[i*2], segmentImageSrv.response.coordinates[i*2+1]);
			//ROS_INFO("sample(%i) relative polar position = %f, %f", i, position[0], position[1]);
			sampleProps.type = i;
			sampleProps.distance = position[0];
			sampleProps.bearing = position[1];
            sampleProps.confidence = imageProbabilitiesSrv.response.responseProbabilities[i];
			searchForSamplesMsgOut.sampleList.push_back(sampleProps);
		}
	}

	gettimeofday(&this->localTimer, NULL);  
    double endSampleLocalizationTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for sample localization: %f", endSampleLocalizationTime - startSampleLocalizationTime);

	/*
		Publish results of search
	*/
    searchForSamplesMsgOut.procType = req.procType;
    searchForSamplesMsgOut.serialNum = req.serialNum;
    searchForSamplesPub.publish(searchForSamplesMsgOut);

	gettimeofday(&this->localTimer, NULL);
	double endSearchTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);
	ROS_INFO("Total time for search: %f", endSearchTime - startSearchTime);

    ros::spinOnce(); //publish results before completing request (important!)
	return true;
}
