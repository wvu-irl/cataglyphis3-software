#include <computer_vision/calibrate_camera.hpp>

CalibrateCamera::CalibrateCamera()
{
    //create window
    namedWindow("Current Image", cv::WINDOW_NORMAL);

    //create trackbars and buttons
    cv::createButton("Show Body\n", bodyCheckBox, (void*)this, CV_CHECKBOX, 1);
    cv::createButton("Show Radius\n", radiusCheckBox, (void*)this, CV_CHECKBOX, 1);
    cv::createButton("Show Pole\n", poleCheckBox, (void*)this, CV_CHECKBOX, 1);
    cv::createButton("Show Grabber\n", grabberCheckBox, (void*)this, CV_CHECKBOX, 1);

    cv::createTrackbar( "body-w", "", &_robotWidthSlider, _robotWidthMax, onTrackbarRobotWidth );
    cv::createTrackbar( "body-h", "", &_robotHeightSlider, _robotHeightMax, onTrackbarRobotHeight );
    cv::createTrackbar( "body-shift", "", &_robotShiftSlider, _robotShiftMax, onTrackbarRobotShift );
    cv::createTrackbar( "radius", "", &_radiusSlider, _radiusMax, onTrackbarRadius );
    cv::createTrackbar( "pole-w", "", &_poleWidthSlider, _poleWidthMax, onTrackbarPoleWidth );
    cv::createTrackbar( "pole-h", "", &_poleHeightSlider, _poleHeightMax, onTrackbarPoleHeight );  

    cv::createButton("Calibrate Pose\n", calibratePoseButton, (void*)this, CV_PUSH_BUTTON, 0);
    cv::createButton("Update Image\n", updateImageButton, (void*)this, CV_PUSH_BUTTON, 0);
    cv::createButton("Load Calibration\n", loadCalibrationButton, (void*)this, CV_PUSH_BUTTON, 0);
    cv::createButton("Save Calibration\n", saveCalibrationButton, (void*)this, CV_PUSH_BUTTON, 0);

    cv::setMouseCallback("Current Image", onMouse, (void*)this);

    //set coordinates for corners of robot
    _globalCoordinates.clear();
    _globalCoordinates.push_back(cv::Point3f(0.315,0.2535,-0.465));
    _globalCoordinates.push_back(cv::Point3f(0.315,-0.2535,-0.465));
    _globalCoordinates.push_back(cv::Point3f(-0.295,0.2535,-0.465));
    _globalCoordinates.push_back(cv::Point3f(-0.295,-0.2535,-0.465));

    //set camera matrix (a11,a12,a13,a21,a22,a23,a31,a32,a33)
    _cameraMatrix = (cv::Mat_<double>(3,3) << 2896.00000, 0.0, 0.0, 0.0, 2896.00000, 0.0, 0.0, 0.0, 1.0);

    //set distortion coefficients (radial1, radial2, tangential1, tangential2, radial3)
    _distortionCoefficients = (cv::Mat_<double>(5,1) << -1.693698799145976e-09, -8.529322941398544e-10, 0.0, 0.0, -4.420202910449835e-09);

}

void CalibrateCamera::displayImage()
{
	// cv::Mat imageUndistorted;
	// undistort(_dst, imageUndistorted, _cameraMatrix, _distortionCoefficients);
    imshow("Current Image", _dst);
}

void CalibrateCamera::updateImage()
{
    //reset calibration mask
    _calibrationMask = cv::Mat::ones(5792,5792,CV_8U)*255;

    //set limits from sliders
    int limitRobotLeftWidth = (_robotWidthMax - _robotWidthSlider)/2;
    int limitRobotRightWidth = (_robotWidthMax - _robotWidthSlider)/2 + _robotWidthSlider;
    int limitRobotHeightTop = _robotHeightMax - _robotHeightSlider - _robotShiftSlider;
    int limitRobotHeightBottom = _robotHeightMax - _robotShiftSlider;
    int limitPoleLeftWidth = (_poleWidthMax - _poleWidthSlider)/2;
    int limitPoleRightWidth = (_poleWidthMax - _poleWidthSlider)/2 + _poleWidthSlider;
    int limitPoleHeight = _poleHeightMax*2 - _poleHeightSlider;

    //update calibration mask from limits
    for(int i=0; i<_calibrationMask.cols; i++)
    {
        for(int j=0; j<_calibrationMask.rows; j++)
        {
            int dist = sqrt( (i-(float)_dst.cols/2)*(i-(float)_dst.cols/2) + (j-(float)_dst.rows/2)*(j-(float)_dst.rows/2) );

            if(_showRadius)
            {
                if(dist > _radiusSlider)
                {
                    _calibrationMask.at<uchar>(j,i)=0;
                }
            }

            if(_showBody)
            {
                if(i > limitRobotLeftWidth && i < limitRobotRightWidth && j > limitRobotHeightTop && j < limitRobotHeightBottom)
                {
                    _calibrationMask.at<uchar>(j,i)=0;
                }
            }

            if(_showPole)
            {
                if(i > limitPoleLeftWidth && i < limitPoleRightWidth && j > limitPoleHeight)
                {
                    _calibrationMask.at<uchar>(j,i)=0;
                }
            }
        }
    }

    //update calibration mask for display and saving to file
    cv::Mat blendMask,calibrationRGB;
    calibrationRGB = _calibrationMask.clone();
    cv::cvtColor(calibrationRGB,calibrationRGB,CV_GRAY2RGB);
    cv::threshold(calibrationRGB,_saveMask,0,255,0);
    cv::threshold(calibrationRGB,blendMask,0,100,1);
    cv::add(_dstCopy,blendMask,_dst);
}

void CalibrateCamera::saveCalibration()
{
    boost::filesystem::path P( ros::package::getPath("computer_vision") );
    cv::imwrite(P.string() + "/data/images/calibration_mask.jpg",_saveMask);
    cv::imwrite(P.string() + "/data/images/calibration_mask_bak.jpg",_saveMask);
}

void CalibrateCamera::findPose()
{
	ROS_INFO("Finding pose...");
	//solve pnp
	cv::Mat rotVec(3,1,cv::DataType<double>::type);
	cv::Mat traVec(3,1,cv::DataType<double>::type);
	cv::solvePnP(_globalCoordinates, _imageCoordinates, _cameraMatrix, _distortionCoefficients, rotVec, traVec, CV_ITERATIVE);
	
	//convert rotVec to rotMat
	ROS_INFO("Converting rotVec to rotMat...");
	cv::Mat rotMat, rotMatT;
	cv::Rodrigues(rotVec,rotMatT);
	cv::transpose(rotMatT,rotMat);
	cv::Mat cameraPose = -rotMat*traVec;

	//extract x, y, z, roll,pitch, and yaw
	ROS_INFO("Extracting x, y, and z...");
	float xCamera=cameraPose.at<double>(0,0);//traVec.at<double>(0,0);
	float yCamera=cameraPose.at<double>(1,0);//traVec.at<double>(1,0);
	float zCamera=cameraPose.at<double>(2,0);//traVec.at<double>(2,0);

	ROS_INFO("Extracting roll, pitch, and yaw...");
	float rollCamera=atan2(-rotMat.at<double>(2,1), rotMat.at<double>(2,2))*180.0/3.14159265;
	float pitchCamera=asin(rotMat.at<double>(2,0))*180.0/3.14159265;
	float yawCamera=atan2(-rotMat.at<double>(1,0), rotMat.at<double>(0,0))*180.0/3.14159265;

	// float rollCamera=atan2(-rotMat.at<double>(1,2), rotMat.at<double>(2,2));
	// float pitchCamera=asin(rotMat.at<double>(0,2));
	// float yawCamera=atan2(-rotMat.at<double>(0,1), rotMat.at<double>(0,0));

	//print result
	ROS_INFO("x = %f",xCamera);
	ROS_INFO("y = %f",yCamera);
	ROS_INFO("z = %f",zCamera);
	ROS_INFO("roll = %f",rollCamera);
	ROS_INFO("pitch = %f",pitchCamera);
	ROS_INFO("yaw = %f",yawCamera);
}