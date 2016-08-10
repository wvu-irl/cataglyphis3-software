#include <computer_vision/calibrate_camera.hpp>

CalibrateCamera::CalibrateCamera()
{
    //create window
    namedWindow("Current Image", cv::WINDOW_NORMAL);

    //create trackbars
    cv::createButton("Show Body\n", bodyCheckBox, (void*)this, CV_CHECKBOX, 0);
    cv::createButton("Show Radius\n", radiusCheckBox, (void*)this, CV_CHECKBOX, 0);
    cv::createButton("Show Pole\n", poleCheckBox, (void*)this, CV_CHECKBOX, 0);
    cv::createButton("Show Grabber\n", grabberCheckBox, (void*)this, CV_CHECKBOX, 0);

    cv::createTrackbar( "body-w", "", &_robotWidthSlider, _robotWidthMax, onTrackbarRobotWidth );
    cv::createTrackbar( "body-h", "", &_robotHeightSlider, _robotHeightMax, onTrackbarRobotHeight );
    cv::createTrackbar( "body-shift", "", &_robotShiftSlider, _robotShiftMax, onTrackbarRobotShift );
    cv::createTrackbar( "radius", "", &_radiusSlider, _radiusMax, onTrackbarRadius );
    cv::createTrackbar( "pole-w", "", &_poleWidthSlider, _poleWidthMax, onTrackbarPoleWidth );
    cv::createTrackbar( "pole-h", "", &_poleHeightSlider, _poleHeightMax, onTrackbarPoleHeight );  

    cv::createButton("Calibrate Pose\n", calibratePoseButton, (void*)this, CV_PUSH_BUTTON, 0);
    cv::createButton("Update Image\n", updateImageButton, (void*)this, CV_PUSH_BUTTON, 0);
    cv::createButton("Save Calibration\n", saveCalibrationButton, (void*)this, CV_PUSH_BUTTON, 0);

    cv::setMouseCallback("Current Image", onMouse, (void*)this);
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
}

void CalibrateCamera::saveCalibration()
{
    boost::filesystem::path P( ros::package::getPath("computer_vision") );
    cv::imwrite(P.string() + "/data/images/calibration_mask.jpg",_saveMask);
    cv::imwrite(P.string() + "/data/images/calibration_mask_bak.jpg",_saveMask);
}