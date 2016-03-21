#include <computer_vision/sample_search.hpp>

SampleSearch::SampleSearch()
{
	// searchForSamplesServ = nh.advertiseService("/vision/samplesearch/searchforsamples", &SampleSearch::searchForSamples, this);
}

// void SampleSearch::initialize_camera()
// {
// 	capture.initialize_camera();
// 	capture.auto_detect_camera();

// 	if(capture.camera_detection_error==0)
// 	{
// 		ROS_INFO("Camera initialized successfully.");	
// 	}
// }

// int SampleSearch::check_camera()
// {
// 	if(capture.camera_detection_error==1)
// 	{
// 		ROS_WARN_THROTTLE(2,"Warning camera connection failed. Trying to detect again...");

// 		capture.initialize_camera();
// 		capture.auto_detect_camera();

// 		ros::Duration(0.02).sleep();
// 		ros::spinOnce();

// 		if(capture.camera_detection_error==0)
// 		{
// 			ROS_INFO("Camera re-initialized successful.");	
// 			return 1;
// 		}
// 		else
// 		{
// 			return 0;
// 		}
// 	}
// }

// void SampleSearch::display_image()
// {

// 	capture.capture_image_to_buffer();
// 	capture.decode_image_from_buffer_to_mat();
// 	imwrite("captured_image.jpg", capture.image_Mat);	
// }

// int SampleSearch::loadLookupTable(std::string filename)
// {
// 	//allocate memory for lookup table
// 	G_lookup_table = new unsigned char**[256];
// 	for (int i = 0; i < 256; ++i) 
// 	{
// 		G_lookup_table[i] = new unsigned char*[256];
// 		for (int j = 0; j < 256; ++j)
// 		{
// 			G_lookup_table[i][j] = new unsigned char[256];
// 		}
// 	}	

// 	//open file containing lookup table
// 	std::ifstream inputFile;
// 	inputFile.open(filename.c_str());
// 	if(!inputFile)
// 	{
// 		ROS_INFO("Error! Failed to open file %s!", filename.c_str());
// 		return -1;
// 	}

// 	//load data in the lookup table
// 	int element = 0;
// 	for (int i=0; i<256; i++) //R
// 	{
// 		for (int j=0; j<256; j++) //G
// 		{
// 			for (int k=0; k<256; k++) //B
// 			{
// 				if(inputFile.eof())
// 				{
// 					ROS_INFO("Error! Unexpected end of file while accessing element (%i, %i) in file %s", i, j, filename.c_str());
// 					return -2;
// 				}
// 				else
// 				{
// 					inputFile >> element;
// 					G_lookup_table[i][j][k] = element;	
// 				}
// 			}
// 		}
// 	}

// 	//close file
// 	inputFile.close();

// 	return 1;
// }

// bool SampleSearch::searchForSamples(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res)
// {
// 	res.confidence =((double)rand() / RAND_MAX);
// 	res.bearing = 0;
// 	res.distance = 1;
// 	return true;
// }
