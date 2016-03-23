#include <computer_vision/sample_search.hpp>

SampleSearch::SampleSearch()
{
	searchForSamplesServ = nh.advertiseService("/vision/samplesearch/searchforsamples", &SampleSearch::searchForSamples, this);
	segmentImageClient = nh.serviceClient<computer_vision::SegmentImage>("/vision/segmentation/segmentimage");
	classifierClient = nh.serviceClient<computer_vision::ImageProbabilities>("/classify_feature_vector_service");
	searchForSamplesPub = nh.advertise<messages::CVSamplesFound>("vision/samplesearch/samplesearchout",1);
}

std::vector<double> SampleSearch::calculateFlatGroundPositionOfPixel(int u, int v)
{
	//transform pixels to center of image
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

bool SampleSearch::searchForSamples(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res)
{
	//call segmentation server
	segmentImageSrv.request.camera = true;
	segmentImageSrv.request.path = "";
	if(segmentImageClient.call(segmentImageSrv))
	{
		ROS_INFO("segmentImageSrv call successful!");
	}
	else
	{
		ROS_ERROR("Error! Failed to call service SegmentImage!");
		return false;
	}

	//call classifier servver
	imageProbabilitiesSrv.request.numBlobs = segmentImageSrv.response.coordinates.size()/2;
	if(classifierClient.call(imageProbabilitiesSrv))
	{
		ROS_INFO("imageProbabilitiesSrv call successful!");
	}
	else
	{
		ROS_ERROR("Error! Failed to call service ImageProbabilities!");
		return false;
	}

	//draw samples and non samples on image
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

	//calculate the position of each sample
	std::vector<double> position;
	messages::CVSampleProps sampleProps;
	searchForSamplesMsgOut.sampleList.clear();
	for(int i=0; i<binary.size(); i++)
	{
		if(binary[i]==1)
		{
			position.clear();
			position = calculateFlatGroundPositionOfPixel(segmentImageSrv.response.coordinates[i*2], segmentImageSrv.response.coordinates[i*2+1]);
			ROS_INFO("sample(%i) relative polar position = %f, %f", i, position[0], position[1]);
			sampleProps.type = i;
			sampleProps.distance = position[0];
			sampleProps.bearing = position[1];
			sampleProps.confidence = imageProbabilitiesSrv.response.responseProbabilities[i]*1000;
			searchForSamplesMsgOut.sampleList.push_back(sampleProps);
		}
	}

	//publish results of search
    searchForSamplesMsgOut.procType = req.procType;
    searchForSamplesMsgOut.serialNum = req.serialNum;
    searchForSamplesPub.publish(searchForSamplesMsgOut);
    ros::spinOnce(); //publish results before completing request (important!)
	return true;
}
