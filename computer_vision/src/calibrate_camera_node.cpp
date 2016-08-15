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
				return 0;
			}
			else
			{
				calibrateCamera._dst = capture.image_Mat.clone();
				calibrateCamera._dstCopy = calibrateCamera._dst.clone();
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
