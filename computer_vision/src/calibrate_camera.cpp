#include <computer_vision/calibrate_camera.hpp>

CalibrateCamera::CalibrateCamera()
{
    namedWindow("Current Image", cv::WINDOW_NORMAL);
}

void CalibrateCamera::setImage(cv::Mat &src)
{
    _dst = src.clone();;
    _dstCopy = src.clone();
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
    cv::createTrackbar( "grab-w", "", &_grabberWidthSlider, _grabberWidthMax, onTrackbarGrabberWidth );
    cv::createTrackbar( "grab-h", "", &_grabberHeightSlider, _grabberHeightMax, onTrackbarGrabberHeight );
    cv::createTrackbar( "grab-shift", "", &_grabberShiftSlider, _grabberShiftMax, onTrackbarGrabberShift );
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
    int limitGrabberLeftWidth = (_grabberWidthMax - _grabberWidthSlider)/2;
    int limitGrabberRightWidth = (_grabberWidthMax - _grabberWidthSlider)/2 + _grabberWidthSlider;
    int limitGrabberHeightTop = _grabberHeightMax - _grabberHeightSlider - _grabberShiftSlider;
    int limitGrabberHeightBottom = _grabberHeightMax - _grabberShiftSlider;

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

            if(_showGrabber)
            {
                if(i > limitGrabberLeftWidth && i < limitGrabberRightWidth && j > limitGrabberHeightTop && j < limitGrabberHeightBottom)
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

	//compensation
	float rollCompensation = (_rollSlider+_rollOffset)/10.0*3.14159265/180.0;
	int pitchCompensationX = 0;
	int pitchCompensationY = -(_pitchSlider+_pitchOffset);
	int yawCompensationX = (_yawSlider+_yawOffset);
	int yawCompensationY = 0;

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

    //calculate attitude with respect to the camera coordinate systems
    int u = (P_V1.x+P_V2.x)/2;
    int v = (P_V1.y+P_V2.y)/2;
    int x = -(v-5792/2);
    int y = u - 5792/2;
    _rollAngle = rollCompensation;
    _pitchAngle = -y*IMAGE_SENSOR_PIXEL_SIZE/G_FOCAL_LENGTH;//atan2(y*IMAGE_SENSOR_PIXEL_SIZE,10e-3);
    _yawAngle = -x*IMAGE_SENSOR_PIXEL_SIZE/G_FOCAL_LENGTH;//atan2(x*IMAGE_SENSOR_PIXEL_SIZE,10e-3);

    if(_showLines)
    {
		//line for local yaw offset
		cv::line(_dst, P_V1, P_V2, cv::Scalar(0,0,255), _LINE_THICKNESS, _LINE_TYPE, _LINE_SHIFT); //vertical
		cv::line(_dst, P_H1, P_H2, cv::Scalar(0,0,255), _LINE_THICKNESS, _LINE_TYPE, _LINE_SHIFT); //horizontal

        //draw circle for distances (NOTE THAT THE CIRCLES ARE NOT THE EXACT REPRESENTATION OF THE DISTANCES)
        float theta1 = atan(0.05/G_SENSOR_HEIGHT);
        float dp1 = G_FOCAL_LENGTH/IMAGE_SENSOR_PIXEL_SIZE*tan(theta1);
        int r1 = dp1;
        cv::circle(_dst,cv::Point(u,v), r1, cv::Scalar(255,0,0), _LINE_THICKNESS, _LINE_TYPE, _LINE_SHIFT);

        float theta2 = atan((0.55)/G_SENSOR_HEIGHT);
        float dp2 = G_FOCAL_LENGTH/IMAGE_SENSOR_PIXEL_SIZE*tan(theta2);
        int r2 = dp2;
        cv::circle(_dst,cv::Point(u,v), r2, cv::Scalar(255,50,0), _LINE_THICKNESS, _LINE_TYPE, _LINE_SHIFT);

        float theta3 = atan((0.55+0.5)/G_SENSOR_HEIGHT);
        float dp3 = G_FOCAL_LENGTH/IMAGE_SENSOR_PIXEL_SIZE*tan(theta3);
        int r3 = dp3;
        cv::circle(_dst,cv::Point(u,v), r3, cv::Scalar(255,100,0), _LINE_THICKNESS, _LINE_TYPE, _LINE_SHIFT);

        float theta4 = atan((0.55+1.0)/G_SENSOR_HEIGHT);
        float dp4 = G_FOCAL_LENGTH/IMAGE_SENSOR_PIXEL_SIZE*tan(theta4);
        int r4 = dp4;
        cv::circle(_dst,cv::Point(u,v), r4, cv::Scalar(255,150,0), _LINE_THICKNESS, _LINE_TYPE, _LINE_SHIFT);

        float theta5 = atan((0.55+1.5)/G_SENSOR_HEIGHT);
        float dp5 = G_FOCAL_LENGTH/IMAGE_SENSOR_PIXEL_SIZE*tan(theta5);
        int r5 = dp5;
        cv::circle(_dst,cv::Point(u,v), r5, cv::Scalar(255,200,0), _LINE_THICKNESS, _LINE_TYPE, _LINE_SHIFT);

        float theta6 = atan((0.55+2.0)/G_SENSOR_HEIGHT);
        float dp6 = G_FOCAL_LENGTH/IMAGE_SENSOR_PIXEL_SIZE*tan(theta6);
        int r6 = dp6;
        cv::circle(_dst,cv::Point(u,v), r6, cv::Scalar(255,255,0), _LINE_THICKNESS, _LINE_TYPE, _LINE_SHIFT);
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
    logger << _rollAngle << std::endl;
    logger << _pitchAngle << std::endl;
    logger << _yawAngle << std::endl;
    logger << _robotWidthSlider << std::endl;
    logger << _robotHeightSlider << std::endl;
    logger << _robotShiftSlider << std::endl;
    logger << _radiusSlider << std::endl;
    logger << _poleWidthSlider << std::endl;
    logger << _poleHeightSlider << std::endl;
    logger << _grabberWidthSlider << std::endl;
    logger << _grabberHeightSlider << std::endl;
    logger << _grabberShiftSlider << std::endl;
    logger << _rollSlider << std::endl;
    logger << _pitchSlider << std::endl;
    logger << _yawSlider << std::endl;
    logger.close();

    //print saved attitude calibration
    ROS_INFO("_rollAngle = %f degrees", _rollAngle*180.0/3.14159265);
    ROS_INFO("_pitchAngle = %f degrees", _pitchAngle*180.0/3.14159265);
    ROS_INFO("_yawAngle = %f degrees", _yawAngle*180.0/3.14159265);
}

void CalibrateCamera::loadCalibration()
{
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

    //load data from calibration file
    inputFile >> _rollAngle;
    inputFile >> _pitchAngle;
    inputFile >> _yawAngle;
    inputFile >> _robotWidthSlider;
    inputFile >> _robotHeightSlider;
    inputFile >> _robotShiftSlider;
    inputFile >> _radiusSlider;
    inputFile >> _poleWidthSlider;
    inputFile >> _poleHeightSlider;
    inputFile >> _grabberWidthSlider;
    inputFile >> _grabberHeightSlider;
    inputFile >> _grabberShiftSlider;
    inputFile >> _rollSlider;
    inputFile >> _pitchSlider;
    inputFile >> _yawSlider;

    //close file
    inputFile.close();

    //print loaded attitude calibration
    ROS_INFO("_rollAngle = %f degrees", _rollAngle*180.0/3.14159265);
    ROS_INFO("_pitchAngle = %f degrees", _pitchAngle*180.0/3.14159265);
    ROS_INFO("_yawAngle = %f degrees", _yawAngle*180.0/3.14159265);
}

////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////CALLBACK FUNCTION IMPLEMENTATIONS/////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

void CalibrateCamera::bodyCheckBoxImplementation(int state)
{
    if(state==true)
    {
        _showBody=true;
    }
    else
    {
        _showBody=false;
    }
}

void CalibrateCamera::poleCheckBoxImplementation(int state)
{
    if(state==true)
    {
        _showPole=true;
    }
    else
    {
        _showPole=false;
    }
}

void CalibrateCamera::grabberCheckBoxImplementation(int state)
{
    if(state==true)
    {
        _showGrabber=true;
    }
    else
    {
        _showGrabber=false;
    }
}

void CalibrateCamera::radiusCheckBoxImplementation(int state)
{
    if(state==true)
    {
        _showRadius=true;
    }
    else
    {
        _showRadius=false;
    }
}

void CalibrateCamera::linesCheckBoxImplementation(int state)
{
    if(state==true)
    {
        _showLines=true;
    }
    else
    {
        _showLines=false;
    }
}

void CalibrateCamera::captureImageButtonImplementation()
{
    _captureImage = true;
}

void CalibrateCamera::updateImageButtonImplementation()
{
    ROS_INFO("Updating image...");
    updateImage();
    ROS_INFO("Complete.");
}

void CalibrateCamera::saveCalibrationButtonImplementation()
{
    ROS_INFO("Calibration parameters saving...");
    saveCalibration();
    ROS_INFO("Complete.");
}

void CalibrateCamera::loadCalibrationButtonImplementation()
{
    ROS_INFO("Loading calibration parameters...");
    loadCalibration();
    updateImage();
    ROS_INFO("Complete.");
    ROS_WARN("Note only the image shows the current calibration (the sliders do not show what is on the image).");
}

void CalibrateCamera::exitButtonImplementation()
{
    ROS_INFO("Exiting program...");
    _exit = true;
}