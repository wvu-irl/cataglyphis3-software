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

std::vector<double> SampleSearch::calculateFlatGroundPositionOfPixel(int u, int v)
{
	//transform pixels to coordinate system centered around the image
	u = u - G_IMAGE_WIDTH/2;
	v = v - G_IMAGE_HEIGHT/2;

	//transform u and v pixels to align with robot x and y 
	double x = -v;
	double y = u;

	//calculate angle between x and y
	double delta_theta = atan2(y,x);

	//calculate angle between z and distance to sample
	float pixel_distance_from_center = sqrt(x*x + y*y);
	double angle = (3.14159265/2)*pixel_distance_from_center/(G_IMAGE_WIDTH/2);//atan2(pixel_distance_from_center*G_PIXEL2DISTANCE,G_FOCAL_LENGTH);

	//calculate distance assuming flat ground tan(angle) = x/z where z is the sensor height
	double distance = G_SENSOR_HEIGHT*tan(angle);

	//transform to center of robot (to cartesian then back to polar)
	double x_t = distance*cos(delta_theta) + 0.4;
	double y_t = distance*sin(delta_theta);
	double distance_t = sqrt(x_t*x_t + y_t*y_t);
	double delta_theta_t = atan2(y,x);

	//put results in vector
	std::vector<double> relative_position;
	relative_position.push_back(distance_t);
	relative_position.push_back(delta_theta_t*180/3.14159265-3);

	//return flat ground position
	return relative_position;
}

void SampleSearch::drawResultsOnImage(const std::vector<int> &binary, const std::vector<int> &coordinates)
{
	ROS_INFO("drawing output image!");

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
		return false;
	}

	gettimeofday(&this->localTimer, NULL);  
    double endSegmentationTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for segmentation service: %f", endSegmentationTime - startSegmentationTime);

	/*
		Call classifier server
	*/
	gettimeofday(&this->localTimer, NULL);
    double startClassifierTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  

	imageProbabilitiesSrv.request.numBlobs = segmentImageSrv.response.coordinates.size()/2;
	imageProbabilitiesSrv.request.imgSize = 150; //50 will do 50x50 classifier, 150 will do 150x150 classifier
	if(classifierClient.call(imageProbabilitiesSrv))
	{
		ROS_INFO("imageProbabilitiesSrv call successful!");
	}
	else
	{
		ROS_ERROR("Error! Failed to call service ImageProbabilities!");
		return false;
	}

	gettimeofday(&this->localTimer, NULL);  
    double endClassifierTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for segmentation service: %f", endClassifierTime - startClassifierTime);

	/*
		Save image, blobs, and blob info if > 0.2 probability of being a sample
	*/
	gettimeofday(&this->localTimer, NULL);
    double startSaveBlobsTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  

	saveLowAndHighProbabilityBlobs(imageProbabilitiesSrv.response.responseProbabilities, segmentImageSrv.response.coordinates);

	gettimeofday(&this->localTimer, NULL);  
    double endSaveBlobsTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for segmentation service: %f", endSaveBlobsTime - startSaveBlobsTime);

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
    ROS_INFO("Time taken in seconds for segmentation service: %f", endDrawTime - startDrawTime);

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
			sampleProps.confidence = imageProbabilitiesSrv.response.responseProbabilities[i]*1000;
			searchForSamplesMsgOut.sampleList.push_back(sampleProps);
		}
	}

	gettimeofday(&this->localTimer, NULL);  
    double endSampleLocalizationTime = this->localTimer.tv_sec+(this->localTimer.tv_usec/1000000.0);  
    ROS_INFO("Time taken in seconds for segmentation service: %f", endSampleLocalizationTime - startSampleLocalizationTime);

	/*
		Publish results of search
	*/
    searchForSamplesMsgOut.procType = req.procType;
    searchForSamplesMsgOut.serialNum = req.serialNum;
    searchForSamplesPub.publish(searchForSamplesMsgOut);
    ros::spinOnce(); //publish results before completing request (important!)
	return true;
}
