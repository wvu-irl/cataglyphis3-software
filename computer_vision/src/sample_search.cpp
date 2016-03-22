#include <computer_vision/sample_search.hpp>

SampleSearch::SampleSearch()
{
	searchForSamplesServ = nh.advertiseService("/vision/samplesearch/searchforsamples", &SampleSearch::searchForSamples, this);
	segmentImageClient = nh.serviceClient<computer_vision::SegmentImage>("/vision/segmentation/segmentimage");
	searchForSamplesPub = nh.advertise<messages::CVSamplesFound>("vision/samplesearch/samplesearchout",1);
}

void SampleSearch::drawResultsOnImage(const std::vector<int> &binary, const std::vector<int> &coordinates)
{
	ROS_INFO("drawing output image!");

	//load image
	boost::filesystem::path P( ros::package::getPath("computer_vision") );
	boost::filesystem::path folder = P / boost::filesystem::path("/data/images/");
	cv::Mat src = cv::imread(folder.string()+"input_image.jpg");

	//draw circle for detected samples and nonsamples
	int idx = 0;
	for(int i=0; i<binary.size(); i++)
	{
		if(binary[i]==1)
		{
			circle(src, cv::Point(coordinates[idx],coordinates[idx+1]), 100, cv::Scalar(0,0,255), 3, 8);
		}
		else
		{
			circle(src, cv::Point(coordinates[idx],coordinates[idx+1]), 100, cv::Scalar(0,0,0), 3, 8);
		}
		idx=idx+2;
	}

	//write image to file
	imwrite(folder.string() + "output_image.jpg", src);
}

bool SampleSearch::searchForSamples(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res)
{
	//call segmentation server
	segmentImageSrv.request.camera = false;
	segmentImageSrv.request.path = "/home/jared/cataglyphis_ws/src/computer_vision/img1.JPG";
	if(segmentImageClient.call(segmentImageSrv))
	{
		ROS_INFO("segmentImageSrv call successful!");
	}
	else
	{
		ROS_ERROR("Error! Failed to call service SegmentImage!");
		return false;
	}

	std::vector<int> binary;
	for(int i=0; i<segmentImageSrv.response.coordinates.size(); i=i+2)
	{
		binary.push_back(0);
	}

	drawResultsOnImage(binary, segmentImageSrv.response.coordinates);

	//publish results of search
    searchForSamplesMsgOut.procType = req.procType;
    searchForSamplesMsgOut.serialNum = req.serialNum;
    searchForSamplesMsgOut.sampleList.clear();
    searchForSamplesPub.publish(searchForSamplesMsgOut);
    ros::spinOnce(); //publish results before completing request
	return true;
}
