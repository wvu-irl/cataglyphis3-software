#include <computer_vision/sample_search.hpp>

SampleSearch::SampleSearch()
{
	searchForSamplesServ = nh.advertiseService("/vision/samplesearch/searchforsamples", &SampleSearch::searchForSamples, this);
}

bool SampleSearch::searchForSamples(computer_vision::SearchForSamples::Request &req, computer_vision::SearchForSamples::Response &res)
{
	res.c = req.a + req.b;
	return true;
}
