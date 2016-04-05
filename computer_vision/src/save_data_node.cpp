#include <ros/ros.h>
#include <image_transport/image_transport.h> 	
#include <cv_bridge/cv_bridge.h> 				
#include <sensor_msgs/image_encodings.h> 
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class Images
{
public:
	image_transport::Subscriber sub_image;

	Images()
	{
			ros::NodeHandle node;
			image_transport::ImageTransport it(node);
			sub_image = it.subscribe("/usb_cam/image_raw/compressed", 1, &Images::imageCallback, this);
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		try
		{
			cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
			cv::waitKey(30);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());

		}

	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "my_node");
	ROS_INFO("my_node running...");
	Images images;
	ros::spin();
	return 0;
} 