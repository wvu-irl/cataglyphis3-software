#include <computer_vision/capture_class.hpp> 
#include <computer_vision/calibrate_camera.hpp>

int main(int argc, char **argv)
{
	//Initialize ROS Node
	ros::init(argc, argv, "calibrate_camera_node");
	ros::Time::init();
	ROS_INFO("calibrate_camera_node running...");

	//Initialize Camera and Caliration Objects
	Capture capture;
	CalibrateCamera calibrateCamera;

	//Capture Initial Image
	if(argc>1)
	{
		boost::filesystem::path P( ros::package::getPath("computer_vision") );
		std::string filename = P.string() + "/f10.jpg";
		cv::Mat tempImage = cv::imread(filename.c_str());
		calibrateCamera.setImage(tempImage);
	}
	else
	{
		if(capture.capture_image()==0)
		{
			ROS_ERROR("Could not capture initial image from camera. Terminating...");
			return 0;
		}
		else
		{
			cv::Mat tempImage = capture.image_Mat.clone();
			calibrateCamera.setImage(tempImage);
		}		
	}
	
	//Main Loop
	calibrateCamera.loadCalibration();
	calibrateCamera.initializeTrackbars();
	calibrateCamera.updateImage();
	while(ros::ok())
	{
		calibrateCamera.displayImage();
		
		char keyPress = cv::waitKey(30);
		if(calibrateCamera._captureImage == true || keyPress == 'r')
		{
			calibrateCamera._captureImage = false;
			if(capture.capture_image()==0)
			{
				ROS_ERROR("Could not capture a new image from camera. Terminating...");
			}
			else
			{
				cv::Mat tempImg = cv::Mat(capture.image_Mat.clone());
				calibrateCamera.setImage(tempImg);
				calibrateCamera.updateImage();
			}
		}
		else if(calibrateCamera._exit == true || keyPress == 'q')
		{
			break;
		}

		ros::spinOnce();
	}
	
	return 0;
} 
