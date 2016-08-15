#include <computer_vision/calibrate_camera.hpp>

CalibrateCamera::CalibrateCamera()
{
    //create window
    namedWindow("Current Image", cv::WINDOW_NORMAL);
    // cv::setMouseCallback("Current Image", onMouse, (void*)this);

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

void CalibrateCamera::initializeTrackbars()
{
    //create trackbars and buttons
    cv::createButton("Show Body\n", bodyCheckBox, (void*)this, CV_CHECKBOX, 1);
    cv::createButton("Show Radius\n", radiusCheckBox, (void*)this, CV_CHECKBOX, 1);
    cv::createButton("Show Pole\n", poleCheckBox, (void*)this, CV_CHECKBOX, 1);
    cv::createButton("Show Grabber\n", grabberCheckBox, (void*)this, CV_CHECKBOX, 1);
    cv::createButton("Show Lines\n", linesCheckBox, (void*)this, CV_CHECKBOX, 1);

    cv::createTrackbar( "body-w", "", &_robotWidthSlider, _robotWidthMax, onTrackbarRobotWidth );
    cv::createTrackbar( "body-h", "", &_robotHeightSlider, _robotHeightMax, onTrackbarRobotHeight );
    cv::createTrackbar( "body-shift", "", &_robotShiftSlider, _robotShiftMax, onTrackbarRobotShift );
    cv::createTrackbar( "radius", "", &_radiusSlider, _radiusMax, onTrackbarRadius );
    cv::createTrackbar( "pole-w", "", &_poleWidthSlider, _poleWidthMax, onTrackbarPoleWidth );
    cv::createTrackbar( "pole-h", "", &_poleHeightSlider, _poleHeightMax, onTrackbarPoleHeight );  
    cv::createTrackbar( "roll-deg", "", &_rollSlider, _rollMax, onTrackbarRoll );
    cv::createTrackbar( "pitch-px", "", &_pitchSlider, _pitchMax, onTrackbarPitch );
    cv::createTrackbar( "yaw-px", "", &_yawSlider, _yawMax, onTrackbarYaw );

    //cv::createButton("Calibrate Pose\n", calibratePoseButton, (void*)this, CV_PUSH_BUTTON, 0);
    cv::createButton("Capture Image\n", captureImageButton, (void*)this, CV_PUSH_BUTTON, 0);
    cv::createButton("Update Image\n", updateImageButton, (void*)this, CV_PUSH_BUTTON, 0);
    cv::createButton("Save Calibration\n", saveCalibrationButton, (void*)this, CV_PUSH_BUTTON, 0);
    cv::createButton("Load Calibration\n", loadCalibrationButton, (void*)this, CV_PUSH_BUTTON, 0);
    cv::createButton("Exit\n", exitButton, (void*)this, CV_PUSH_BUTTON, 0);
}

void CalibrateCamera::displayImage()
{
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

 	if(_showLines)
	{
		//compensation
		float rollCompensation = (_rollSlider+_rollOffset)/10.0*3.14159265/180.0;
		int pitchCompensationX = 0;
		int pitchCompensationY = -(_pitchSlider+_pitchOffset);
		int yawCompensationX = (_yawSlider+_yawOffset);
		int yawCompensationY = 0;

		cv::Mat R = (cv::Mat_<double>(2,2) << cos(rollCompensation), sin(rollCompensation), -sin(rollCompensation), cos(rollCompensation));

		//vertical line
		cv::Point P_V1 = cv::Point( 5792/2, 5792 ); //endpoint1 of line segment
		P_V1 = cv::Point( P_V1.x - 5792/2, P_V1.y - 5792/2 ); //transform from image to robot
		P_V1 = cv::Point(P_V1.x*cos(-rollCompensation) + P_V1.y*sin(-rollCompensation), -P_V1.x*sin(-rollCompensation) + P_V1.y*cos(-rollCompensation)); //roll rotation
		P_V1 = cv::Point( P_V1.x + 5792/2, P_V1.y + 5792/2 ); //transform from robot to image
		P_V1 = cv::Point(P_V1.x + pitchCompensationX + yawCompensationX, P_V1.y + pitchCompensationY + yawCompensationY); //pitch and yaw rotation


		cv::Point P_V2 = cv::Point(5792/2, 0);
		P_V2 = cv::Point( P_V2.x - 5792/2, P_V2.y - 5792/2 ); //transform from image to robot
		P_V2 = cv::Point(P_V2.x*cos(-rollCompensation) + P_V2.y*sin(-rollCompensation), -P_V2.x*sin(-rollCompensation) + P_V2.y*cos(-rollCompensation)); //roll rotation
		P_V2 = cv::Point( P_V2.x + 5792/2, P_V2.y + 5792/2 ); //transform from robot to image
		P_V2 = cv::Point(P_V2.x + pitchCompensationX + yawCompensationX, P_V2.y + pitchCompensationY + yawCompensationY); //pitch and yaw rotation

		//horizontal line
		cv::Point P_H1 = cv::Point( 5792, 5792/2 );
		P_H1 = cv::Point( P_H1.x - 5792/2, P_H1.y - 5792/2 ); //transform from image to robot
		P_H1 = cv::Point(P_H1.x*cos(-rollCompensation) + P_H1.y*sin(-rollCompensation), -P_H1.x*sin(-rollCompensation) + P_H1.y*cos(-rollCompensation)); //roll rotation
		P_H1 = cv::Point( P_H1.x + 5792/2, P_H1.y + 5792/2 ); //transform from robot to image
		P_H1 = cv::Point(P_H1.x + pitchCompensationX + yawCompensationX, P_H1.y + pitchCompensationY + yawCompensationY); //pitch and yaw rotation

		cv::Point P_H2 = cv::Point( 0, 5792/2);
		P_H2 = cv::Point( P_H2.x - 5792/2, P_H2.y - 5792/2 ); //transform from image to robot
		P_H2 = cv::Point(P_H2.x*cos(-rollCompensation) + P_H2.y*sin(-rollCompensation), -P_H2.x*sin(-rollCompensation) + P_H2.y*cos(-rollCompensation)); //roll rotation
		P_H2 = cv::Point( P_H2.x + 5792/2, P_H2.y + 5792/2 ); //transform from robot to image
		P_H2 = cv::Point(P_H2.x + pitchCompensationX + yawCompensationX, P_H2.y + pitchCompensationY + yawCompensationY); //pitch and yaw rotation

		//line for local yaw offset
		cv::line(_dst, P_V1, P_V2, cv::Scalar(0,0,255), _LINE_THICKNESS, _LINE_TYPE, _LINE_SHIFT); //vertical
		cv::line(_dst, P_H1, P_H2, cv::Scalar(0,0,255), _LINE_THICKNESS, _LINE_TYPE, _LINE_SHIFT); //horizontal

		//convert to meaningful stuff
		ROS_INFO("angle = %f", rollCompensation*180.0/3.14159265);
	    int u = (P_V1.x+P_V2.x)/2;
	    int v = (P_V1.y+P_V2.y)/2;
	    ROS_INFO("u,v = %i,%i",u,v);
	    int x = -(v-5792/2);
	    int y = u - 5792/2;
	    ROS_INFO("x,y = %i,%i",x,y);
	}
}

void CalibrateCamera::saveCalibration()
{
    //write calibration image to file
    boost::filesystem::path P( ros::package::getPath("computer_vision") );
    cv::imwrite(P.string() + "/data/calibration/calibration_mask.jpg",_saveMask);

    //write calibration parameters to file
    std::ofstream logger;
    std::string filename = P.string() + "/data/calibration/cameraCalibration.csv";
    logger.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc); 
    logger << _robotWidthSlider << std::endl;
    logger << _robotHeightSlider << std::endl;
    logger << _robotShiftSlider << std::endl;
    logger << _radiusSlider << std::endl;
    logger << _poleWidthSlider << std::endl;
    logger << _poleHeightSlider << std::endl;
    logger << _rollSlider << std::endl;
    logger << _rollOffset << std::endl;
    logger << _pitchSlider << std::endl;
    logger << _pitchOffset << std::endl;
    logger << _yawSlider << std::endl;
    logger << _yawOffset;
    logger.close();
}

void CalibrateCamera::loadCalibration()
{
    ROS_ERROR("Loading calibration file...");

    //get path to calibration parameters
    boost::filesystem::path P( ros::package::getPath("computer_vision") );
    std::string filename = P.string() + "/data/calibration/cameraCalibration.csv";

    //open file containing calibration parameters
    std::ifstream inputFile;
    inputFile.open(filename.c_str());
    if(!inputFile)
    {
        ROS_ERROR("Error! Failed to open calibration file.");
        return;
    }

    //load data in the lookup table
    int element = 0;

    while(!inputFile.eof())
    {
        inputFile >> _robotWidthSlider;
        inputFile >> _robotHeightSlider;
        inputFile >> _robotShiftSlider;
        inputFile >> _radiusSlider;
        inputFile >> _poleWidthSlider;
        inputFile >> _poleHeightSlider;
        inputFile >> _rollSlider;
        inputFile >> _rollOffset;
        inputFile >> _pitchSlider;
        inputFile >> _pitchOffset;
        inputFile >> _yawSlider;
        inputFile >> _yawOffset;
    }

    //close file
    inputFile.close();

    ROS_ERROR("Complete.");
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

    ROS_INFO("Complete.");
}

/////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////BUTTON CALLBACK FUNCTION IMPLEMENTATION///////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

// void CalibrateCamera::bodyCheckBoxImplementation()
// {

// }

// void CalibrateCamera::poleCheckBoxImplementation()
// {

// }

// void CalibrateCamera::grabberCheckBoxImplementation()
// {

// }

// void CalibrateCamera::radiusCheckBoxImplementation()
// {

// }

// void CalibrateCamera::linesCheckBoxImplementation()
// {

// }

