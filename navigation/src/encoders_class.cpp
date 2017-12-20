/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <navigation/encoders_class.hpp>

Encoders::Encoders()
{
	// counter_front=0;
	// counter_back=0;
	// counter_middle=0;
	counter_left=0;
	counter_right=0;
	counter_left_prev=0;
	counter_right_prev=0;
	fl=0;
	fr=0;
	bl=0;
	br=0;
	ml=0;
	mr=0;
	fl_prev=0;
	fr_prev=0;
	bl_prev=0;
	br_prev=0;
	ml_prev=0;
	mr_prev=0;
	fl_diff=0;
	fr_diff=0;
	bl_diff=0;
	br_diff=0;
	ml_diff=0;
	mr_diff=0;
	encoder_max_count=61566;
	impossible_encoder_diff=10000;
	spike_diff = 1000;
	wheel_radius=0;
	counts_per_revolution=1;
	// subscriber_encoder_front = node.subscribe("roboteq/drivemotorin/front", 1, &Encoders::getEncoderFrontCallback,this);
	// subscriber_encoder_back = node.subscribe("roboteq/drivemotorin/back", 1, &Encoders::getEncoderBackCallback,this);
	// subscriber_encoder_middle = node.subscribe("roboteq/drivemotorin/middle", 1, &Encoders::getEncoderMiddleCallback,this);
	subscriber_encoder_left = node.subscribe("roboteq/drivemotorin/left", 1, &Encoders::getEncoderLeftCallback,this);
	subscriber_encoder_right = node.subscribe("roboteq/drivemotorin/right", 1, &Encoders::getEncoderRightCallback,this);
}

void Encoders::set_wheel_radius(double set_radius)
{
	wheel_radius=set_radius;
}

void Encoders::set_counts_per_revolution(double set_counts)
{
	counts_per_revolution=set_counts;
}

void Encoders::adjustEncoderWrapError()
{
	if(abs(fl_diff)>impossible_encoder_diff)
	{
		if (fl_prev > encoder_max_count/2) fl_diff = (encoder_max_count-fl_prev)+fl;
		else fl_diff = -fl_prev-(encoder_max_count-fl);
		if(abs(fl_diff)>spike_diff) fl_diff = 0;
	}

	if(abs(fr_diff)>impossible_encoder_diff)
	{
		if (fr_prev > encoder_max_count/2) fr_diff = (encoder_max_count-fr_prev)+fr;
		else fr_diff = -fr_prev-(encoder_max_count-fr);
		if(abs(fr_diff)>spike_diff) fr_diff = 0;
	}

	if(abs(bl_diff)>impossible_encoder_diff)
	{
		if (bl_prev > encoder_max_count/2) bl_diff = (encoder_max_count-bl_prev)+bl;
		else bl_diff = -bl_prev-(encoder_max_count-bl);
		if(abs(bl_diff)>spike_diff) bl_diff = 0;
	}

	if(abs(br_diff)>impossible_encoder_diff)
	{
		if (br_prev > encoder_max_count/2) br_diff = (encoder_max_count-br_prev)+br;
		else br_diff = -br_prev-(encoder_max_count-br);
		if(abs(br_diff)>spike_diff) br_diff = 0;
	}

	if(abs(ml_diff)>impossible_encoder_diff)
	{
		if (ml_prev > encoder_max_count/2) ml_diff = (encoder_max_count-ml_prev)+ml;
		else ml_diff = -ml_prev-(encoder_max_count-ml);
		if(abs(ml_diff)>spike_diff) ml_diff = 0;
	}

	if(abs(mr_diff)>impossible_encoder_diff)
	{
		if (mr_prev > encoder_max_count/2) mr_diff = (encoder_max_count-mr_prev)+mr;
		else mr_diff = -mr_prev-(encoder_max_count-mr);
		if(abs(mr_diff)>spike_diff) mr_diff = 0;
	}
}

void Encoders::calculateWheelDistancesFromEncoders()
{
	if(this->counter_left_prev != this->counter_left)
	{
		fl_dist = (double)fl_diff/(double)counts_per_revolution*2.0*wheel_radius*3.14159265;
		bl_dist = (double)bl_diff/(double)counts_per_revolution*2.0*wheel_radius*3.14159265;
		ml_dist = (double)ml_diff/(double)counts_per_revolution*2.0*wheel_radius*3.14159265;
	}
	else
	{
		fl_dist=0;
		bl_dist=0;
		ml_dist=0;
	}
	this->counter_left_prev = this->counter_left;

	if(this->counter_right_prev != this->counter_right)
	{
		fr_dist = (double)fr_diff/(double)counts_per_revolution*2.0*wheel_radius*3.14159265;
		br_dist = (double)br_diff/(double)counts_per_revolution*2.0*wheel_radius*3.14159265;
		mr_dist = (double)mr_diff/(double)counts_per_revolution*2.0*wheel_radius*3.14159265;
	}
	else
	{
		fr_dist=0;
		br_dist=0;
		mr_dist=0;
	}
	this->counter_right_prev = this->counter_right;
}

void Encoders::calculateDeltaDistance4Wheels(int turnFlag, int stopFlag)
{
	short int logical = (1-stopFlag)*(1-turnFlag);
	delta_distance  = (double)logical*1/2*( (fl_dist+bl_dist)/2 + (fr_dist+br_dist)/2 );
}

void Encoders::calculateDeltaDistance6Wheels(int turnFlag, int stopFlag)
{
	double med_left = 0;
	double med_right = 0;
	if ((ml_dist>=fl_dist && ml_dist<=bl_dist)||(ml_dist<=fl_dist && ml_dist>=bl_dist))
	{
		med_left = ml_dist;
	}
	else if ((fl_dist>=ml_dist && fl_dist<=bl_dist)||(fl_dist<=ml_dist && fl_dist>=bl_dist))
	{
		med_left = fl_dist;
	}
	else
	{
		med_left = bl_dist;
	}

	if ((mr_dist>=fr_dist && mr_dist<=br_dist)||(mr_dist<=fr_dist && mr_dist>=br_dist))
	{
		med_right = mr_dist;
	}
	else if ((fr_dist>=mr_dist && fr_dist<=br_dist)||(fr_dist<=mr_dist && fr_dist>=br_dist))
	{
		med_right = fr_dist;
	}
	else
	{
		med_right = br_dist;
	}

	short int logical = (1-stopFlag)*(1-turnFlag);
	delta_distance  = (double)logical*(0.5)*(med_left + med_right);
}
