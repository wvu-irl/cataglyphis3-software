#include <computer_vision/capture_class.hpp> 
#include <computer_vision/calibrate_camera.hpp>

int main(int argc, char **argv)
{
	//Initialize ROS Node
	ros::init(argc, argv, "calibrate_camera_node");
	ROS_INFO("calibrate_camera_node running...");

	//Initialize Camera and Caliration Objects
	Capture capture;
	CalibrateCamera calibrateCamera;

	//Capture Initial Image
	calibrateCamera._dst = cv::imread("/home/jared/cataglyphis_ws/src/computer_vision/f10.jpg");
	calibrateCamera._dstCopy = calibrateCamera._dst.clone();

	// if(capture.capture_image()==0)
	// {
	// 	ROS_ERROR("Could not capture initial image from camera. Terminating...");
	// 	return 0;
	// }
	// else
	// {
	// 	dst = capture.image_Mat.clone();
	// 	dstCopy = dst.clone();
	// }

	//Main Loop
	while(ros::ok())
	{
		calibrateCamera.displayImage();
		
		char keyPress = cv::waitKey(30);
		if(keyPress == 'r')
		{
			if(capture.capture_image()==0)
			{
				ROS_ERROR("Could not capture a new image from camera. Terminating...");
				return 0;
			}
			else
			{
				calibrateCamera._dst = capture.image_Mat.clone();
				calibrateCamera._dstCopy = calibrateCamera._dst.clone();
			}
		}
		else if(keyPress == 'u')
		{
			ROS_INFO("Updating image...");
			calibrateCamera.updateImage();

		}
		else if(keyPress == 's')
		{
		    ROS_INFO("Writing new mask to file...");
			calibrateCamera.saveCalibration();
		}
		else if(keyPress == 'q')
		{
			break;
		}

		ros::spinOnce();
	}
	
	return 0;
} 
