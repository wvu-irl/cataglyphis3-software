#include <ros/ros.h>
#include <computer_vision/segmentation.hpp>
#include <ros/package.h>
#include <boost/filesystem.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "segmentation_node");
	ROS_INFO("segmentation_node running...");
	Segmentation segmentation;
	segmentation.setCalibration();
	boost::filesystem::path P( ros::package::getPath("computer_vision") );
	segmentation.loadLookupTable(P.string() + "/lookup.csv");
	ros::spin();
	return 0;
}
