#include <computer_vision/sample_search.hpp>

SampleSearch::SampleSearch()
{
	searchForSamplesServ = nh.advertiseService("/vision/samplesearch/searchforsamples", &SampleSearch::searchForSamples, this);
	search_pub = nh.advertise<messages::CVSamplesFound>("vision/samplesearch/samplesearchout",1);
}

bool SampleSearch::searchForSamples(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res)
{
    msg_cv_samples_found.procType = req.procType;
    msg_cv_samples_found.serialNum = req.serialNum;
    msg_cv_samples_found.sampleList.clear();
    search_pub.publish(msg_cv_samples_found);
    ros::spinOnce();
	return true;
}
