#include <ros/ros.h>
#include <computer_vision/sample_search.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "computer_vision_node");
	ROS_INFO("computer_vision_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100); 

	SampleSearch samplesSearch;
	samplesSearch.initialize_camera();

	//boost::filesystem::path P( ros::package::getPath("computer_vision") );
	//samplesSearch.loadLookupTable(P.string() + "/lookup.csv");

	//while(ros::ok())
	//{
		samplesSearch.check_camera();
		samplesSearch.display_image();

		//ros::spinOnce();
		//loop_rate.sleep();
	//}

	return 0;
} 