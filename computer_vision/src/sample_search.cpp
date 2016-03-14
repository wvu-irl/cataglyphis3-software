#include <computer_vision/sample_search.hpp>

SampleSearch::SampleSearch()
{
	searchForSamplesServ = nh.advertiseService("/vision/samplesearch/searchforsamples", &SampleSearch::searchForSamples, this);
}

int SampleSearch::loadLookupTable(std::string filename)
{
	//allocate memory for lookup table
	G_lookup_table = new unsigned char**[256];
	for (int i = 0; i < 256; ++i) 
	{
		G_lookup_table[i] = new unsigned char*[256];
		for (int j = 0; j < 256; ++j)
		{
			G_lookup_table[i][j] = new unsigned char[256];
		}
	}	

	//open file containing lookup table
	std::ifstream inputFile;
	inputFile.open(filename.c_str());
	if(!inputFile)
	{
		ROS_INFO("Error! Failed to open file %s!", filename.c_str());
		return -1;
	}

	//load data in the lookup table
	int element = 0;
	for (int i=0; i<256; i++) //R
	{
		for (int j=0; j<256; j++) //G
		{
			for (int k=0; k<256; k++) //B
			{
				if(inputFile.eof())
				{
					ROS_INFO("Error! Unexpected end of file while accessing element (%i, %i) in file %s", i, j, filename.c_str());
					return -2;
				}
				else
				{
					inputFile >> element;
					G_lookup_table[i][j][k] = element;	
				}
			}
		}
	}

	//close file
	inputFile.close();

	return 1;
}

bool SampleSearch::searchForSamples(computer_vision::SearchForSamples::Request &req, computer_vision::SearchForSamples::Response &res)
{
	res.c = req.a + req.b;
	return true;
}
