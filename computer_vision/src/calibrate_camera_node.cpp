#include <ros/ros.h>
#include <ros/package.h>
#include <computer_vision/capture_class.hpp> 
#include <computer_vision/calibrate_camera.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <boost/filesystem.hpp>

//Robot Width Image
int robotWidthMax = 0;
int robotWidthSlider = 0;

//Robot Height in Image
int robotHeightMax = 0;
int robotHeightSlider = 0;

//Robot Shift in Image
int robotShiftMax = 0;
int robotShiftSlider = 0;

//Blocked Radius in Image
int radiusMax = 0;
int radiusSlider = 0;

//Pole Width Image
int poleWidthMax = 0;
int poleWidthSlider = 0;

//Pole Height Image
int poleHeightMax = 0;
int poleHeightSlider = 0;

// Matrices to store images
cv::Mat dst;
cv::Mat dstCopy;

void onTrackbarRobotWidth( int, void* )
{
	// ROS_INFO("robotWidthSlider = %i",robotWidthSlider);
}

void onTrackbarRobotHeight( int, void* )
{
	// ROS_INFO("robotHeightSlider = %i",robotHeightSlider);
}

void onTrackbarRobotShift( int, void* )
{
	// ROS_INFO("robotShiftSlider = %i",robotShiftSlider);
}

void onTrackbarRadius( int, void* )
{
	// ROS_INFO("radiusSlider = %i",radiusSlider);
}

void onTrackbarPoleWidth( int, void* )
{
	// ROS_INFO("poleWidthSlider = %i",poleWidthSlider);
}

void onTrackbarPoleHeight( int, void* )
{
	// ROS_INFO("poleHeightSlider = %i",poleHeightSlider);
}

int main(int argc, char **argv)
{
	//Initialize ROS Node
	ros::init(argc, argv, "calibrate_camera_node");
	ROS_INFO("calibrate_camera_node running...");

	//Initialize Camera and Caliration Objects
	Capture capture;
	CalibrateCamera calibrateCamera;

	//Capture Initial Image
	dst = cv::imread("/home/jared/cataglyphis_ws/src/computer_vision/samples.jpg");
	dstCopy = dst.clone();

	//Set Maximum Values for Sliders
	robotWidthMax = dst.cols;
	robotHeightMax = dst.rows;
	robotShiftMax = dst.rows;
	radiusMax = sqrt((float)(robotWidthMax/2*robotWidthMax/2)+(float)(robotHeightMax/2*robotHeightMax/2));
	radiusSlider = radiusMax;
	poleWidthMax = dst.cols;
	poleHeightMax = dst.rows/2;

	//Create Windows
	namedWindow("Current Image", cv::WINDOW_NORMAL);

	//Create Trackbars
	char TrackbarRobotWidth[50];
	sprintf( TrackbarRobotWidth, "RW", robotWidthMax );
	cv::createTrackbar( TrackbarRobotWidth, "Current Image", &robotWidthSlider, robotWidthMax, onTrackbarRobotWidth );

	char TrackbarRobotHeight[50];
	sprintf( TrackbarRobotHeight, "RH", robotHeightMax );
	cv::createTrackbar( TrackbarRobotHeight, "Current Image", &robotHeightSlider, robotHeightMax, onTrackbarRobotHeight );

	char TrackbarRobotShift[50];
	sprintf( TrackbarRobotShift, "RS", robotShiftMax );
	cv::createTrackbar( TrackbarRobotShift, "Current Image", &robotShiftSlider, robotShiftMax, onTrackbarRobotShift );

	char TrackbarRadius[50];
	sprintf( TrackbarRadius, "BR", radiusMax );
	cv::createTrackbar( TrackbarRadius, "Current Image", &radiusSlider, radiusMax, onTrackbarRadius );

	char TrackbarPoleWidth[50];
	sprintf( TrackbarPoleWidth, "PW", poleWidthMax );
	cv::createTrackbar( TrackbarPoleWidth, "Current Image", &poleWidthSlider, poleWidthMax, onTrackbarPoleWidth );

	char TrackbarPoleHeight[50];
	sprintf( TrackbarPoleHeight, "PH", poleHeightMax );
	cv::createTrackbar( TrackbarPoleHeight, "Current Image", &poleHeightSlider, poleHeightMax, onTrackbarPoleHeight );

	//Initialize calibration image
	cv::Mat calibrationMask = cv::Mat::zeros(5792,5792,CV_8U);

	//Main Loop
	while(ros::ok())
	{
		imshow("Current Image", dst);

		char keyPress = cv::waitKey(30);
		if(keyPress == 'r')
		{
			if(capture.capture_image()==1)
			{
				ROS_ERROR("Could not capture a new image from camera...");
			}
			else
			{
				calibrateCamera.refreshImage(capture.image_Mat);
			}
		}
		else if(keyPress == 'u')
		{
			ROS_INFO("Updating image...");
    		int limitRobotLeftWidth = (robotWidthMax - robotWidthSlider)/2;
    		int limitRobotRightWidth = (robotWidthMax - robotWidthSlider)/2 + robotWidthSlider;
    		int limitRobotHeightTop = robotHeightMax - robotHeightSlider - robotShiftSlider;
    		int limitRobotHeightBottom = robotHeightMax - robotShiftSlider;

    		int limitPoleLeftWidth = (poleWidthMax - poleWidthSlider)/2;
    		int limitPoleRightWidth = (poleWidthMax - poleWidthSlider)/2 + poleWidthSlider;
    		int limitPoleHeight = poleHeightMax*2 - poleHeightSlider;

		    for(int i=0; i<calibrationMask.cols; i++)
		    {
		        for(int j=0; j<calibrationMask.rows; j++)
		        {
					int dist = sqrt( (i-(float)dst.cols/2)*(i-(float)dst.cols/2) + (j-(float)dst.rows/2)*(j-(float)dst.rows/2) );
					if(dist < radiusSlider)
					{
						if(i < limitRobotLeftWidth || i > limitRobotRightWidth || j < limitRobotHeightTop || j > limitRobotHeightBottom)
						{
							if(i < limitPoleLeftWidth || i > limitPoleRightWidth || j < limitPoleHeight)
							{
								calibrationMask.at<uchar>(j,i)=255;
							}
						}
					}
		        }
		    }

		   //invert and threshold mask for display display masked image
		    cv::Mat displayMask,blendMask,calibrationRGB,blended;
		    calibrationRGB = calibrationMask.clone();
		    cv::cvtColor(calibrationRGB,calibrationRGB,CV_GRAY2RGB);
		    cv::threshold(calibrationRGB,displayMask,0,255,0);
			cv::threshold(calibrationRGB,blendMask,0,100,1);
			blended = cv::Mat::zeros(5792,5792,CV_8U);
			cv::add(dstCopy,blendMask,dst);
		}
		else if(keyPress == 's')
		{
			boost::filesystem::path P( ros::package::getPath("computer_vision") );
			cv::imwrite(P.string() + "/data/images/calibration_mask.jpg",calibrationMask);
			cv::imwrite(P.string() + "/data/images/calibration_mask_bak.jpg",calibrationMask);
		}
		else if(keyPress == 'q')
		{
			break;
		}

		ros::spinOnce();
	}
	
	return 0;
} 
