#include <computer_vision/sample_search.hpp>

SampleSearch::SampleSearch()
{
	searchForSamplesServ = nh.advertiseService("/vision/samplesearch/searchforsamples", &SampleSearch::searchForSamples, this);
	segmentImageClient = nh.serviceClient<computer_vision::SegmentImage>("/vision/segmentation/segmentimage");
	searchForSamplesPub = nh.advertise<messages::CVSamplesFound>("vision/samplesearch/samplesearchout",1);
}

int SampleSearch::drawResultsOnImage(const std::vector<int> &binary, const std::vector<int> &u, const std::vector<int> &v, cv::Mat &I)
{
	//draw circle for detected samples and nonsamples
	cv::Mat src = I.clone();

	for(int i=0; i<binary.size(); i++)
	{
		if(binary[i]==1)
		{
			circle(src, cv::Point(u[i],v[i]), 32, cv::Scalar(0,0,255), 3, 8);
		}
		else
		{
			circle(src, cv::Point(u[i],v[i]), 32, cv::Scalar(0,0,0), 3, 8);
		}
	}

	//write image to file
	boost::filesystem::path P( ros::package::getPath("computer_vision") );
	boost::filesystem::path folder = P / boost::filesystem::path("/data/images/");
	remove_all(folder);
	boost::filesystem::create_directory(folder);
	imwrite(folder.string() + "samples.jpg", src);
}

bool SampleSearch::searchForSamples(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res)
{
	//call segmentation server
	segmentImageSrv.request.camera = true;
	segmentImageSrv.request.path = "";
	if(segmentImageClient.call(segmentImageSrv))
	{
		ROS_INFO("segmentImageSrv call successful!");
		return true;
	}
	else
	{
		ROS_ERROR("Error! Failed to call service SegmentImage!");
		return false;
	}

	//publish results of search
    searchForSamplesMsgOut.procType = req.procType;
    searchForSamplesMsgOut.serialNum = req.serialNum;
    searchForSamplesMsgOut.sampleList.clear();
    searchForSamplesPub.publish(searchForSamplesMsgOut);
    ros::spinOnce(); //publish results before completing request
	return true;
}
