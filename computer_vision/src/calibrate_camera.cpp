#include <computer_vision/calibrate_camera.hpp>

CalibrateCamera::CalibrateCamera()
{

}

bool CalibrateCamera::refreshImage(cv::Mat src)
{
    if(src.rows!=5792 || src.cols!=5792)
    {
        ROS_ERROR("Error! The captured image should be 5792 rows and 5792 columns. The image captured currently has %i columns and %i rows.",src.cols,src.rows);
        return false;
    }  
    else
    {
    	_inputImage = src.clone();
    	return true;
    }
}