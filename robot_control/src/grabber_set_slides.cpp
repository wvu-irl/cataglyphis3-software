#include <robot_control/grabber_set_slides.h>

void GrabberSetSlides::init()
{
	slidePos_ = params.int1;
}

int GrabberSetSlides::run()
{
    ROS_INFO("slidePos = %f",slidePos_);
	robotOutputs.slidePosCmd = slidePos_;
	robotOutputs.grabberStopCmd = 0;
	slideStatusLEL_.LE_Latch(robotStatus.grabberSlideStatus);
	if(slideStatusLEL_.get_val()) return 1;
	else if(abs(robotStatus.grabberSlidePos - slidePos_) <= slideTol_) return 1;
	else return 0;
}
