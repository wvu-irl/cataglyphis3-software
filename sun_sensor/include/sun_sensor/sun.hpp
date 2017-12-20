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

#ifndef CALENDAR_CLASS_HPP
#define CALENDAR_CLASS_HPP

#include <ros/ros.h>
#include <armadillo>

const double pi = 3.141592653589793238;

class Sun
{
public:
	struct Calendar
	{
		double year;
		double month;
		double day;
		double hour;
		double minute;
		double second;
	};
	Calendar calendar;

	double time_year;
	double time_month;
	double time_day;
	double time_hour;
	double time_min;
	double time_sec;
	double time_UTC;
	double location_latitude;
	double location_longitude;
	double location_altitude;
	double julian_day;
	double julian_ephemeris_day;
	double julian_century;
	double julian_ephemeris_century;
	double julian_ephemeris_millenium;
	double earth_heliocentric_position_latitude;
	double earth_heliocentric_position_longitude;
	double earth_heliocentric_position_radius;
	double sun_heliocentric_position_latitude;
	double sun_heliocentric_position_longitude;
	double sun_geocentric_position_latitude;
	double sun_geocentric_position_longitude;
	double nutation_longitude;
	double nutation_obliquity;
	double true_obliquity;
	double abberation_correction;
	double apparent_sun_longitude;
	double apparent_stime_at_greenwich;
	double sun_rigth_ascension;
	double sun_geocentric_declination;
	double sun_zenith;
	double sun_azimuth;
	double observer_local_hour;
	double topocentric_sun_position_rigth_ascension_parallax;
	double topocentric_sun_position_rigth_ascension;
	double topocentric_sun_position_declination;
	double topocentric_local_hour;

	Sun();
	void get_calendar();
	void julian_calculation();
	void earth_heliocentric_position_calculation();
	void sun_geocentric_position_calculation();
	void nutation_calculation();
	void true_obliquity_calculation();
	void abberation_correction_calculation();
	void apparent_sun_longitude_calculation();
	void apparent_stime_at_greenwich_calculation();
	void sun_rigth_ascension_calculation();
	void sun_geocentric_declination_calculation();
	void observer_local_hour_calculation();
	void topocentric_sun_position_calculate();
	void topocentric_local_hour_calculate();
	void sun_topocentric_zenith_angle_calculate();
	void sun_position(double UTC, double latitude, double longitude, double altitude);
};

#endif //CALENDAR_CLASS_HPP
