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

#include "sun_sensor/sun.hpp"

Sun::Sun()
{
	time_year=0;
	time_month=0;
	time_day=0;
	time_hour=0;
	time_min=0;
	time_sec=0;
	time_UTC=0;
	location_latitude=0;
	location_longitude=0;
	location_altitude=0;
	julian_day=0;
	julian_ephemeris_day=0;
	julian_century=0;
	julian_ephemeris_century=0;
	julian_ephemeris_millenium=0;
	earth_heliocentric_position_latitude=0;
	earth_heliocentric_position_longitude=0;
	earth_heliocentric_position_radius=0;
	sun_heliocentric_position_latitude=0;
	sun_heliocentric_position_longitude=0;
	sun_geocentric_position_latitude=0;
	sun_geocentric_position_longitude=0;
	nutation_longitude=0;
	nutation_obliquity=0;
	true_obliquity=0;
	abberation_correction=0;
	apparent_sun_longitude=0;
	apparent_stime_at_greenwich=0;
	sun_rigth_ascension=0;
	sun_geocentric_declination=0;
	sun_zenith=0;
	sun_azimuth=0;
	observer_local_hour=0;
	topocentric_sun_position_rigth_ascension_parallax=0;
	topocentric_sun_position_rigth_ascension=0;
	topocentric_sun_position_declination=0;
	topocentric_local_hour=0;

}

void Sun::get_calendar()
{
    time_t now = time(0);
    tm *ltm = localtime(&now);

    calendar.year = 1900 + ltm->tm_year;
    calendar.month = 1 + ltm->tm_mon;
    calendar.day = ltm->tm_mday;
    calendar.hour = ltm->tm_hour;
    calendar.minute = 1 + ltm->tm_min;
    calendar.second = 1 + ltm->tm_sec;
}

void Sun::sun_position(double UTC, double latitude, double longitude, double altitude)
{
	// This .cpp function was converted from the matlab script sun = sun_position(time, location), which had the following description:
	//
	// This function compute the sun position (zenith and azimuth angle at the observer
	// location) as a function of the observer local time and position.
	//
	// It is an implementation of the algorithm presented by Reda et Andreas in:
	//   Reda, I., Andreas, A. (2003) Solar position algorithm for solar
	//   radiation application. National Renewable Energy Laboratory (NREL)
	//   Technical report NREL/TP-560-34302.
	// This document is avalaible at www.osti.gov/bridge
	//
	// This algorithm is based on numerical approximation of the exact equations.
	// The authors of the original paper state that this algorithm should be
	// precise at +/- 0.0003 degrees. I have compared it to NOAA solar table
	// (http://www.srrb.noaa.gov/highlights/sunrise/azel.html) and to USNO solar
	// table (http://aa.usno.navy.mil/data/docs/AltAz.html) and found very good
	// correspondance (up to the precision of those tables), except for large
	// zenith angle, where the refraction by the atmosphere is significant
	// (difference of about 1 degree). Note that in this code the correction
	// for refraction in the atmosphere as been implemented for a temperature
	// of 10C (283 kelvins) and a pressure of 1010 mbar. See the subfunction
	// «sun_topocentric_zenith_angle_calculation» for a possible modification
	// to explicitely model the effect of temperature and pressure as describe
	// in Reda & Andreas (2003).
	//
	// Input parameters:
	//   time: a structure that specify the time when the sun position is
	//   calculated.
	//       time.year: year. Valid for [-2000, 6000]
	//       time.month: month [1-12]
	//       time.day: calendar day [1-31]
	//       time.hour: local hour [0-23]
	//       time.min: minute [0-59]
	//       time.sec: second [0-59]
	//       time.UTC: offset hour from UTC. Local time = Greenwich time + time.UTC
	//   This input can also be passed using the Matlab time format ('dd-mmm-yyyy HH:MM:SS').
	//   In that case, the time has to be specified as UTC time (time.UTC = 0)
	//
	//   location: a structure that specify the location of the observer
	//       location.latitude: latitude (in degrees, north of equator is
	//       positive)
	//       location.longitude: longitude (in degrees, positive for east of
	//       Greenwich)
	//       location.altitude: altitude above mean sea level (in meters)
	//
	// Output parameters
	//   sun: a structure with the calculated sun position
	//       sun.zenith = zenith angle in degrees (angle from the vertical)
	//       sun.azimuth = azimuth angle in degrees, eastward from the north.
	// Only the sun zenith and azimuth angles are returned as output, but a lot
	// of other parameters are calculated that could also extracted as output of
	// this function.
	//
	// Exemple of use
	//
	// location.longitude = -105.1786;
	// location.latitude = 39.742476;
	// location.altitude = 1830.14;
	// time.year = 2003;
	// time.month = 10;
	// time.day = 17;
	// time.hour = 12;
	// time.min = 30;
	// time.sec = 30;
	// time.UTC = -7;
	//
	// sun = sun_position(time, location);
	//
	// sun =
	//
	//      zenith: 50.1080438859849
	//      azimuth: 194.341174010338
	//
	// History
	//   09/03/2004  Original creation by Vincent Roy (vincent.roy@drdc-rddc.gc.ca)
	//   10/03/2004  Fixed a bug in julian_calculation subfunction (was
	//               incorrect for year 1582 only), Vincent Roy
	//   18/03/2004  Correction to the header (help display) only. No changes to
	//               the code. (changed the «elevation» field in «location» structure
	//               information to «altitude»), Vincent Roy
	//   13/04/2004  Following a suggestion from Jody Klymak (jklymak@ucsd.edu),
	//               allowed the 'time' input to be passed as a Matlab time string.
	//   22/08/2005  Following a bug report from Bruce Bowler
	//               (bbowler@bigelow.org), modified the julian_calculation function. Bug
	//               was 'MATLAB has allowed structure assignment  to a non-empty non-structure
	//               to overwrite the previous value.  This behavior will continue in this release,
	//               but will be an error in a  future version of MATLAB.  For advice on how to
	//               write code that  will both avoid this warning and work in future versions of
	//               MATLAB,  see R14SP2 Release Notes'. Script should now be
	//               compliant with futher release of Matlab...
	time_year=calendar.year;
	time_month=calendar.month;
	time_day=calendar.day;
	time_hour=calendar.hour;
	time_min=calendar.minute;
	time_sec=calendar.second;
	time_UTC=UTC;
	location_latitude=latitude;
	location_longitude=longitude;
	location_altitude=altitude;

	// 1. Calculate the Julian Day, and Century. Julian Ephemeris day, century
	// and millenium are calculated using a mean delta_t of 33.184 seconds.
	julian_calculation();

	// 2. Calculate the Earth heliocentric longitude, latitude, and radius
	// vector (L, B, and R)
	earth_heliocentric_position_calculation();

	// 3. Calculate the geocentric longitude and latitude
	sun_geocentric_position_calculation();

	// 4. Calculate the nutation in longitude and obliquity (in degrees).
	nutation_calculation();

	// 5. Calculate the true obliquity of the ecliptic (in degrees).
	true_obliquity_calculation();

	// 6. Calculate the aberration correction (in degrees)
	abberation_correction_calculation();

	// 7. Calculate the apparent sun longitude in degrees)
	apparent_sun_longitude_calculation();

	// 8. Calculate the apparent sideral time at Greenwich (in degrees)
	apparent_stime_at_greenwich_calculation();

	// 9. Calculate the sun rigth ascension (in degrees)
	sun_rigth_ascension_calculation();

	// 10. Calculate the geocentric sun declination (in degrees). Positive or
	// negative if the sun is north or south of the celestial equator.
	sun_geocentric_declination_calculation();

	// 11. Calculate the observer local hour angle (in degrees, westward from south).
	observer_local_hour_calculation();

	// 12. Calculate the topocentric sun position (rigth ascension, declination and
	// rigth ascension parallax in degrees)
	topocentric_sun_position_calculate();

	// 13. Calculate the topocentric local hour angle (in degrees)
	topocentric_local_hour_calculate();

	// 14. Calculate the topocentric zenith and azimuth angle (in degrees)
	sun_topocentric_zenith_angle_calculate();

}



	//////////////////////////////////////////////////////
	// Subfunction definitions //
	//////////////////////////////////////////////////////




void Sun::julian_calculation()
{
	// This function compute the julian day and julian century from the local
	// time and timezone information. Ephemeris are calculated with a delta_t=0
	// seconds.

	// If time input is a Matlab time string, extract the information from
	// this string and create the structure as defined in the main header of
	// this script.
	double Y;
	double M;
	if(time_month == 1 | time_month == 2)
	{
		Y = time_year - 1;
		M = time_month + 12;
	}
	else
	{
		Y = time_year;
		M = time_month;
	}
	double ut_time = ((time_hour - time_UTC)/24) + (time_min/(60*24)) + (time_sec/(60*60*24)); // time of day in UT time.
	double D = time_day + ut_time; // Day of month in decimal time, ex. 2sd day of month at 12:30:30UT, D=2.521180556

	double A = 0;
	double B = 0;
	// In 1582, the gregorian calendar was adopted
	if(time_year == 1582)
	{
		if(time_month == 10)
		{
			if(time_day <= 4) // The Julian calendar ended on October 4, 1582
			{
				B = 0;
			}
			else if(time_day >= 15) // The Gregorian calendar started on October 15, 1582
			{
				A = floor(Y/100);
				B = 2 - A + floor(A/4);
			}
			else
			{
				time_month = 10;
				time_day = 4;
				B = 0;
			}
		}
		else if(time_month<10) // Julian calendar
		{
			B = 0;
		}
		else // Gregorian calendar
		{
			A = floor(Y/100);
			B = 2 - A + floor(A/4);
		}
	}
	else if(time_year<1582) // Julian calendar
	{
		B = 0;
	}
	else
	{
		A = floor(Y/100); // Gregorian calendar
		B = 2 - A + floor(A/4);
	}

	julian_day = floor(365.25*(Y+4716)) + floor(30.6001*(M+1)) + D + B - 1524.5;

	double delta_t = 0; // 33.184;
	julian_ephemeris_day = julian_day + (delta_t/86400);

	julian_century = (julian_day - 2451545) / 36525;

	julian_ephemeris_century = (julian_ephemeris_day - 2451545) / 36525;

	julian_ephemeris_millenium = julian_ephemeris_century / 10;
}

void Sun::earth_heliocentric_position_calculation()
{
	// This function compute the earth position relative to the sun, using
	// tabulated values.

	// Tabulated values for the longitude calculation
	// L terms  from the original code.
	arma::mat L0_terms(64,3);
	L0_terms(0, 0) = 175347046.000000;
	L0_terms(0, 1) = 0.000000;
	L0_terms(0, 2) = 0.000000;
	L0_terms(1, 0) = 3341656.000000;
	L0_terms(1, 1) = 4.669257;
	L0_terms(1, 2) = 6283.075850;
	L0_terms(2, 0) = 34894.000000;
	L0_terms(2, 1) = 4.626100;
	L0_terms(2, 2) = 12566.151700;
	L0_terms(3, 0) = 3497.000000;
	L0_terms(3, 1) = 2.744100;
	L0_terms(3, 2) = 5753.384900;
	L0_terms(4, 0) = 3418.000000;
	L0_terms(4, 1) = 2.828900;
	L0_terms(4, 2) = 3.523100;
	L0_terms(5, 0) = 3136.000000;
	L0_terms(5, 1) = 3.627700;
	L0_terms(5, 2) = 77713.771500;
	L0_terms(6, 0) = 2676.000000;
	L0_terms(6, 1) = 4.418100;
	L0_terms(6, 2) = 7860.419400;
	L0_terms(7, 0) = 2343.000000;
	L0_terms(7, 1) = 6.135200;
	L0_terms(7, 2) = 3930.209700;
	L0_terms(8, 0) = 1324.000000;
	L0_terms(8, 1) = 0.742500;
	L0_terms(8, 2) = 11506.769800;
	L0_terms(9, 0) = 1273.000000;
	L0_terms(9, 1) = 2.037100;
	L0_terms(9, 2) = 529.691000;
	L0_terms(10, 0) = 1199.000000;
	L0_terms(10, 1) = 1.109600;
	L0_terms(10, 2) = 1577.343500;
	L0_terms(11, 0) = 990.000000;
	L0_terms(11, 1) = 5.233000;
	L0_terms(11, 2) = 5884.927000;
	L0_terms(12, 0) = 902.000000;
	L0_terms(12, 1) = 2.045000;
	L0_terms(12, 2) = 26.298000;
	L0_terms(13, 0) = 857.000000;
	L0_terms(13, 1) = 3.508000;
	L0_terms(13, 2) = 398.149000;
	L0_terms(14, 0) = 780.000000;
	L0_terms(14, 1) = 1.179000;
	L0_terms(14, 2) = 5223.694000;
	L0_terms(15, 0) = 753.000000;
	L0_terms(15, 1) = 2.533000;
	L0_terms(15, 2) = 5507.553000;
	L0_terms(16, 0) = 505.000000;
	L0_terms(16, 1) = 4.583000;
	L0_terms(16, 2) = 18849.228000;
	L0_terms(17, 0) = 492.000000;
	L0_terms(17, 1) = 4.205000;
	L0_terms(17, 2) = 775.523000;
	L0_terms(18, 0) = 357.000000;
	L0_terms(18, 1) = 2.920000;
	L0_terms(18, 2) = 0.067000;
	L0_terms(19, 0) = 317.000000;
	L0_terms(19, 1) = 5.849000;
	L0_terms(19, 2) = 11790.629000;
	L0_terms(20, 0) = 284.000000;
	L0_terms(20, 1) = 1.899000;
	L0_terms(20, 2) = 796.298000;
	L0_terms(21, 0) = 271.000000;
	L0_terms(21, 1) = 0.315000;
	L0_terms(21, 2) = 10977.079000;
	L0_terms(22, 0) = 243.000000;
	L0_terms(22, 1) = 0.345000;
	L0_terms(22, 2) = 5486.778000;
	L0_terms(23, 0) = 206.000000;
	L0_terms(23, 1) = 4.806000;
	L0_terms(23, 2) = 2544.314000;
	L0_terms(24, 0) = 205.000000;
	L0_terms(24, 1) = 1.869000;
	L0_terms(24, 2) = 5573.143000;
	L0_terms(25, 0) = 202.000000;
	L0_terms(25, 1) = 2.445800;
	L0_terms(25, 2) = 6069.777000;
	L0_terms(26, 0) = 156.000000;
	L0_terms(26, 1) = 0.833000;
	L0_terms(26, 2) = 213.299000;
	L0_terms(27, 0) = 132.000000;
	L0_terms(27, 1) = 3.411000;
	L0_terms(27, 2) = 2942.463000;
	L0_terms(28, 0) = 126.000000;
	L0_terms(28, 1) = 1.083000;
	L0_terms(28, 2) = 20.775000;
	L0_terms(29, 0) = 115.000000;
	L0_terms(29, 1) = 0.645000;
	L0_terms(29, 2) = 0.980000;
	L0_terms(30, 0) = 103.000000;
	L0_terms(30, 1) = 0.636000;
	L0_terms(30, 2) = 4694.003000;
	L0_terms(31, 0) = 102.000000;
	L0_terms(31, 1) = 0.976000;
	L0_terms(31, 2) = 15720.839000;
	L0_terms(32, 0) = 102.000000;
	L0_terms(32, 1) = 4.267000;
	L0_terms(32, 2) = 7.114000;
	L0_terms(33, 0) = 99.000000;
	L0_terms(33, 1) = 6.210000;
	L0_terms(33, 2) = 2146.170000;
	L0_terms(34, 0) = 98.000000;
	L0_terms(34, 1) = 0.680000;
	L0_terms(34, 2) = 155.420000;
	L0_terms(35, 0) = 86.000000;
	L0_terms(35, 1) = 5.980000;
	L0_terms(35, 2) = 161000.690000;
	L0_terms(36, 0) = 85.000000;
	L0_terms(36, 1) = 1.300000;
	L0_terms(36, 2) = 6275.960000;
	L0_terms(37, 0) = 85.000000;
	L0_terms(37, 1) = 3.670000;
	L0_terms(37, 2) = 71430.700000;
	L0_terms(38, 0) = 80.000000;
	L0_terms(38, 1) = 1.810000;
	L0_terms(38, 2) = 17260.150000;
	L0_terms(39, 0) = 79.000000;
	L0_terms(39, 1) = 3.040000;
	L0_terms(39, 2) = 12036.460000;
	L0_terms(40, 0) = 71.000000;
	L0_terms(40, 1) = 1.760000;
	L0_terms(40, 2) = 5088.630000;
	L0_terms(41, 0) = 74.000000;
	L0_terms(41, 1) = 3.500000;
	L0_terms(41, 2) = 3154.690000;
	L0_terms(42, 0) = 74.000000;
	L0_terms(42, 1) = 4.680000;
	L0_terms(42, 2) = 801.820000;
	L0_terms(43, 0) = 70.000000;
	L0_terms(43, 1) = 0.830000;
	L0_terms(43, 2) = 9437.760000;
	L0_terms(44, 0) = 62.000000;
	L0_terms(44, 1) = 3.980000;
	L0_terms(44, 2) = 8827.390000;
	L0_terms(45, 0) = 61.000000;
	L0_terms(45, 1) = 1.820000;
	L0_terms(45, 2) = 7084.900000;
	L0_terms(46, 0) = 57.000000;
	L0_terms(46, 1) = 2.780000;
	L0_terms(46, 2) = 6286.600000;
	L0_terms(47, 0) = 56.000000;
	L0_terms(47, 1) = 4.390000;
	L0_terms(47, 2) = 14143.500000;
	L0_terms(48, 0) = 56.000000;
	L0_terms(48, 1) = 3.470000;
	L0_terms(48, 2) = 6279.550000;
	L0_terms(49, 0) = 52.000000;
	L0_terms(49, 1) = 0.190000;
	L0_terms(49, 2) = 12139.550000;
	L0_terms(50, 0) = 52.000000;
	L0_terms(50, 1) = 1.330000;
	L0_terms(50, 2) = 1748.020000;
	L0_terms(51, 0) = 51.000000;
	L0_terms(51, 1) = 0.280000;
	L0_terms(51, 2) = 5856.480000;
	L0_terms(52, 0) = 49.000000;
	L0_terms(52, 1) = 0.490000;
	L0_terms(52, 2) = 1194.450000;
	L0_terms(53, 0) = 41.000000;
	L0_terms(53, 1) = 5.370000;
	L0_terms(53, 2) = 8429.240000;
	L0_terms(54, 0) = 41.000000;
	L0_terms(54, 1) = 2.400000;
	L0_terms(54, 2) = 19651.050000;
	L0_terms(55, 0) = 39.000000;
	L0_terms(55, 1) = 6.170000;
	L0_terms(55, 2) = 10447.390000;
	L0_terms(56, 0) = 37.000000;
	L0_terms(56, 1) = 6.040000;
	L0_terms(56, 2) = 10213.290000;
	L0_terms(57, 0) = 37.000000;
	L0_terms(57, 1) = 2.570000;
	L0_terms(57, 2) = 1059.380000;
	L0_terms(58, 0) = 36.000000;
	L0_terms(58, 1) = 1.710000;
	L0_terms(58, 2) = 2352.870000;
	L0_terms(59, 0) = 36.000000;
	L0_terms(59, 1) = 1.780000;
	L0_terms(59, 2) = 6812.770000;
	L0_terms(60, 0) = 33.000000;
	L0_terms(60, 1) = 0.590000;
	L0_terms(60, 2) = 17789.850000;
	L0_terms(61, 0) = 30.000000;
	L0_terms(61, 1) = 0.440000;
	L0_terms(61, 2) = 83996.850000;
	L0_terms(62, 0) = 30.000000;
	L0_terms(62, 1) = 2.740000;
	L0_terms(62, 2) = 1349.870000;
	L0_terms(63, 0) = 25.000000;
	L0_terms(63, 1) = 3.160000;
	L0_terms(63, 2) = 4690.480000;

	arma::mat L1_terms(34,3);
	L1_terms(0, 0) = 628331966747.000000;
	L1_terms(0, 1) = 0.000000;
	L1_terms(0, 2) = 0.000000;
	L1_terms(1, 0) = 206059.000000;
	L1_terms(1, 1) = 2.678235;
	L1_terms(1, 2) = 6283.075850;
	L1_terms(2, 0) = 4303.000000;
	L1_terms(2, 1) = 2.635100;
	L1_terms(2, 2) = 12566.151700;
	L1_terms(3, 0) = 425.000000;
	L1_terms(3, 1) = 1.590000;
	L1_terms(3, 2) = 3.523000;
	L1_terms(4, 0) = 119.000000;
	L1_terms(4, 1) = 5.796000;
	L1_terms(4, 2) = 26.298000;
	L1_terms(5, 0) = 109.000000;
	L1_terms(5, 1) = 2.966000;
	L1_terms(5, 2) = 1577.344000;
	L1_terms(6, 0) = 93.000000;
	L1_terms(6, 1) = 2.590000;
	L1_terms(6, 2) = 18849.230000;
	L1_terms(7, 0) = 72.000000;
	L1_terms(7, 1) = 1.140000;
	L1_terms(7, 2) = 529.690000;
	L1_terms(8, 0) = 68.000000;
	L1_terms(8, 1) = 1.870000;
	L1_terms(8, 2) = 398.150000;
	L1_terms(9, 0) = 67.000000;
	L1_terms(9, 1) = 4.410000;
	L1_terms(9, 2) = 5507.550000;
	L1_terms(10, 0) = 59.000000;
	L1_terms(10, 1) = 2.890000;
	L1_terms(10, 2) = 5223.690000;
	L1_terms(11, 0) = 56.000000;
	L1_terms(11, 1) = 2.170000;
	L1_terms(11, 2) = 155.420000;
	L1_terms(12, 0) = 45.000000;
	L1_terms(12, 1) = 0.400000;
	L1_terms(12, 2) = 796.300000;
	L1_terms(13, 0) = 36.000000;
	L1_terms(13, 1) = 0.470000;
	L1_terms(13, 2) = 775.520000;
	L1_terms(14, 0) = 29.000000;
	L1_terms(14, 1) = 2.650000;
	L1_terms(14, 2) = 7.110000;
	L1_terms(15, 0) = 21.000000;
	L1_terms(15, 1) = 5.340000;
	L1_terms(15, 2) = 0.980000;
	L1_terms(16, 0) = 19.000000;
	L1_terms(16, 1) = 1.850000;
	L1_terms(16, 2) = 5486.780000;
	L1_terms(17, 0) = 19.000000;
	L1_terms(17, 1) = 4.970000;
	L1_terms(17, 2) = 213.300000;
	L1_terms(18, 0) = 17.000000;
	L1_terms(18, 1) = 2.990000;
	L1_terms(18, 2) = 6275.960000;
	L1_terms(19, 0) = 16.000000;
	L1_terms(19, 1) = 0.030000;
	L1_terms(19, 2) = 2544.310000;
	L1_terms(20, 0) = 16.000000;
	L1_terms(20, 1) = 1.430000;
	L1_terms(20, 2) = 2146.170000;
	L1_terms(21, 0) = 15.000000;
	L1_terms(21, 1) = 1.210000;
	L1_terms(21, 2) = 10977.080000;
	L1_terms(22, 0) = 12.000000;
	L1_terms(22, 1) = 2.830000;
	L1_terms(22, 2) = 1748.020000;
	L1_terms(23, 0) = 12.000000;
	L1_terms(23, 1) = 3.260000;
	L1_terms(23, 2) = 5088.630000;
	L1_terms(24, 0) = 12.000000;
	L1_terms(24, 1) = 5.270000;
	L1_terms(24, 2) = 1194.450000;
	L1_terms(25, 0) = 12.000000;
	L1_terms(25, 1) = 2.080000;
	L1_terms(25, 2) = 4694.000000;
	L1_terms(26, 0) = 11.000000;
	L1_terms(26, 1) = 0.770000;
	L1_terms(26, 2) = 553.570000;
	L1_terms(27, 0) = 10.000000;
	L1_terms(27, 1) = 1.300000;
	L1_terms(27, 2) = 3286.600000;
	L1_terms(28, 0) = 10.000000;
	L1_terms(28, 1) = 4.240000;
	L1_terms(28, 2) = 1349.870000;
	L1_terms(29, 0) = 9.000000;
	L1_terms(29, 1) = 2.700000;
	L1_terms(29, 2) = 242.730000;
	L1_terms(30, 0) = 9.000000;
	L1_terms(30, 1) = 5.640000;
	L1_terms(30, 2) = 951.720000;
	L1_terms(31, 0) = 8.000000;
	L1_terms(31, 1) = 5.300000;
	L1_terms(31, 2) = 2352.870000;
	L1_terms(32, 0) = 6.000000;
	L1_terms(32, 1) = 2.650000;
	L1_terms(32, 2) = 9437.760000;
	L1_terms(33, 0) = 6.000000;
	L1_terms(33, 1) = 4.670000;
	L1_terms(33, 2) = 4690.480000;

	arma::mat L2_terms(20,3);
	L2_terms(0, 0) = 52919.000000;
	L2_terms(0, 1) = 0.000000;
	L2_terms(0, 2) = 0.000000;
	L2_terms(1, 0) = 8720.000000;
	L2_terms(1, 1) = 1.072100;
	L2_terms(1, 2) = 6283.075800;
	L2_terms(2, 0) = 309.000000;
	L2_terms(2, 1) = 0.867000;
	L2_terms(2, 2) = 12566.152000;
	L2_terms(3, 0) = 27.000000;
	L2_terms(3, 1) = 0.050000;
	L2_terms(3, 2) = 3.520000;
	L2_terms(4, 0) = 16.000000;
	L2_terms(4, 1) = 5.190000;
	L2_terms(4, 2) = 26.300000;
	L2_terms(5, 0) = 16.000000;
	L2_terms(5, 1) = 3.680000;
	L2_terms(5, 2) = 155.420000;
	L2_terms(6, 0) = 10.000000;
	L2_terms(6, 1) = 0.760000;
	L2_terms(6, 2) = 18849.230000;
	L2_terms(7, 0) = 9.000000;
	L2_terms(7, 1) = 2.060000;
	L2_terms(7, 2) = 77713.770000;
	L2_terms(8, 0) = 7.000000;
	L2_terms(8, 1) = 0.830000;
	L2_terms(8, 2) = 775.520000;
	L2_terms(9, 0) = 5.000000;
	L2_terms(9, 1) = 4.660000;
	L2_terms(9, 2) = 1577.340000;
	L2_terms(10, 0) = 4.000000;
	L2_terms(10, 1) = 1.030000;
	L2_terms(10, 2) = 7.110000;
	L2_terms(11, 0) = 4.000000;
	L2_terms(11, 1) = 3.440000;
	L2_terms(11, 2) = 5573.140000;
	L2_terms(12, 0) = 3.000000;
	L2_terms(12, 1) = 5.140000;
	L2_terms(12, 2) = 796.300000;
	L2_terms(13, 0) = 3.000000;
	L2_terms(13, 1) = 6.050000;
	L2_terms(13, 2) = 5507.550000;
	L2_terms(14, 0) = 3.000000;
	L2_terms(14, 1) = 1.190000;
	L2_terms(14, 2) = 242.730000;
	L2_terms(15, 0) = 3.000000;
	L2_terms(15, 1) = 6.120000;
	L2_terms(15, 2) = 529.690000;
	L2_terms(16, 0) = 3.000000;
	L2_terms(16, 1) = 0.310000;
	L2_terms(16, 2) = 398.150000;
	L2_terms(17, 0) = 3.000000;
	L2_terms(17, 1) = 2.280000;
	L2_terms(17, 2) = 553.570000;
	L2_terms(18, 0) = 2.000000;
	L2_terms(18, 1) = 4.380000;
	L2_terms(18, 2) = 5223.690000;
	L2_terms(19, 0) = 2.000000;
	L2_terms(19, 1) = 3.750000;
	L2_terms(19, 2) = 0.980000;

	arma::mat L3_terms(7,3);
	L3_terms(0, 0) = 289.000000;
	L3_terms(0, 1) = 5.844000;
	L3_terms(0, 2) = 6283.076000;
	L3_terms(1, 0) = 35.000000;
	L3_terms(1, 1) = 0.000000;
	L3_terms(1, 2) = 0.000000;
	L3_terms(2, 0) = 17.000000;
	L3_terms(2, 1) = 5.490000;
	L3_terms(2, 2) = 12566.150000;
	L3_terms(3, 0) = 3.000000;
	L3_terms(3, 1) = 5.200000;
	L3_terms(3, 2) = 155.420000;
	L3_terms(4, 0) = 1.000000;
	L3_terms(4, 1) = 4.720000;
	L3_terms(4, 2) = 3.520000;
	L3_terms(5, 0) = 1.000000;
	L3_terms(5, 1) = 5.300000;
	L3_terms(5, 2) = 18849.230000;
	L3_terms(6, 0) = 1.000000;
	L3_terms(6, 1) = 5.970000;
	L3_terms(6, 2) = 242.730000;

	arma::mat L4_terms(3,3);
	L4_terms(0, 0) = 114.000000;
	L4_terms(0, 1) = 3.142000;
	L4_terms(0, 2) = 0.000000;
	L4_terms(1, 0) = 8.000000;
	L4_terms(1, 1) = 4.130000;
	L4_terms(1, 2) = 6283.080000;
	L4_terms(2, 0) = 1.000000;
	L4_terms(2, 1) = 3.840000;
	L4_terms(2, 2) = 12566.150000;

	arma::mat L5_terms(1, 3);
	L5_terms(0, 0) = 1.000000;
	L5_terms(0, 1) = 3.140000;
	L5_terms(0, 2) = 0.000000;

	arma::mat A0 = L0_terms.col(0);
	arma::mat B0 = L0_terms.col(1);
	arma::mat C0 = L0_terms.col(2);

	arma::mat A1 = L1_terms.col(0);
	arma::mat B1 = L1_terms.col(1);
	arma::mat C1 = L1_terms.col(2);

	arma::mat A2 = L2_terms.col(0);
	arma::mat B2 = L2_terms.col(1);
	arma::mat C2 = L2_terms.col(2);

	arma::mat A3 = L3_terms.col(0);
	arma::mat B3 = L3_terms.col(1);
	arma::mat C3 = L3_terms.col(2);

	arma::mat A4 = L4_terms.col(0);
	arma::mat B4 = L4_terms.col(1);
	arma::mat C4 = L4_terms.col(2);

	arma::mat A5 = L5_terms.col(0);
	arma::mat B5 = L5_terms.col(1);
	arma::mat C5 = L5_terms.col(2);

	double JME = julian_ephemeris_millenium;
	// Compute the Earth Heliochentric longitude from the tabulated values.
	double L0 = arma::sum(arma::sum(A0 % arma::cos(B0 + (C0 * JME))));
	double L1 = arma::sum(arma::sum(A1 % arma::cos(B1 + (C1 * JME))));
	double L2 = arma::sum(arma::sum(A2 % arma::cos(B2 + (C2 * JME))));
	double L3 = arma::sum(arma::sum(A3 % arma::cos(B3 + (C3 * JME))));
	double L4 = arma::sum(arma::sum(A4 % arma::cos(B4 + (C4 * JME))));
	arma::mat L5_mat = A5 % arma::cos(B5 + (C5 * JME));
	double L5 = L5_mat(0,0);

	earth_heliocentric_position_longitude = (L0 + (L1 * JME) + (L2 * JME*JME) + (L3 * JME*JME*JME) + (L4 * JME*JME*JME*JME) + (L5 * JME*JME*JME*JME*JME)) / 1e8;
	// Convert the longitude to degrees.
	earth_heliocentric_position_longitude = earth_heliocentric_position_longitude * 180/pi;
	// Limit the range to [0,360[;
	earth_heliocentric_position_longitude = fmod(earth_heliocentric_position_longitude, 360.0);
	if (earth_heliocentric_position_longitude<0)
	{
		earth_heliocentric_position_longitude=earth_heliocentric_position_longitude+360.0;
	}

	// Tabulated values for the earth heliocentric latitude.
	// B terms  from the original code.
	arma::mat B0_terms(5, 3);
	B0_terms(0, 0) = 280.000000;
	B0_terms(0, 1) = 3.199000;
	B0_terms(0, 2) = 84334.662000;
	B0_terms(1, 0) = 102.000000;
	B0_terms(1, 1) = 5.422000;
	B0_terms(1, 2) = 5507.553000;
	B0_terms(2, 0) = 80.000000;
	B0_terms(2, 1) = 3.880000;
	B0_terms(2, 2) = 5223.690000;
	B0_terms(3, 0) = 44.000000;
	B0_terms(3, 1) = 3.700000;
	B0_terms(3, 2) = 2352.870000;
	B0_terms(4, 0) = 32.000000;
	B0_terms(4, 1) = 4.000000;
	B0_terms(4, 2) = 1577.340000;

	arma::mat B1_terms(2, 3);
	B1_terms(0, 0) = 9.000000;
	B1_terms(0, 1) = 3.900000;
	B1_terms(0, 2) = 5507.550000;
	B1_terms(1, 0) = 6.000000;
	B1_terms(1, 1) = 1.730000;
	B1_terms(1, 2) = 5223.690000;

	A0.clear();
	B0.clear();
	C0.clear();

	A1.clear();
	B1.clear();
	C1.clear();

	A0 = B0_terms.col(0);
	B0 = B0_terms.col(1);
	C0 = B0_terms.col(2);

	A1 = B1_terms.col(0);
	B1 = B1_terms.col(1);
	C1 = B1_terms.col(2);

	L0 = arma::sum(arma::sum(A0 % arma::cos(B0 + (C0 * JME))));
	L1 = arma::sum(arma::sum(A1 % arma::cos(B1 + (C1 * JME))));

	earth_heliocentric_position_latitude = (L0 + (L1 * JME)) / 1e8;
	// Convert the latitude to degrees.
	earth_heliocentric_position_latitude = earth_heliocentric_position_latitude * 180/pi;
	// Limit the range to [0,360];
	earth_heliocentric_position_latitude = fmod(earth_heliocentric_position_latitude, 360.0);
	if (earth_heliocentric_position_latitude<0)
	{
		earth_heliocentric_position_latitude = earth_heliocentric_position_latitude+360.0;
	}

	// Tabulated values for radius vector.
	// R terms from the original code
	arma::mat R0_terms(40, 3);
	R0_terms(0, 0) = 100013989.000000;
	R0_terms(0, 1) = 0.000000;
	R0_terms(0, 2) = 0.000000;
	R0_terms(1, 0) = 1670700.000000;
	R0_terms(1, 1) = 3.098463;
	R0_terms(1, 2) = 6283.075850;
	R0_terms(2, 0) = 13956.000000;
	R0_terms(2, 1) = 3.055250;
	R0_terms(2, 2) = 12566.151700;
	R0_terms(3, 0) = 3084.000000;
	R0_terms(3, 1) = 5.198500;
	R0_terms(3, 2) = 77713.771500;
	R0_terms(4, 0) = 1628.000000;
	R0_terms(4, 1) = 1.173900;
	R0_terms(4, 2) = 5753.384900;
	R0_terms(5, 0) = 1576.000000;
	R0_terms(5, 1) = 2.846900;
	R0_terms(5, 2) = 7860.419400;
	R0_terms(6, 0) = 925.000000;
	R0_terms(6, 1) = 5.453000;
	R0_terms(6, 2) = 11506.770000;
	R0_terms(7, 0) = 542.000000;
	R0_terms(7, 1) = 4.564000;
	R0_terms(7, 2) = 3930.210000;
	R0_terms(8, 0) = 472.000000;
	R0_terms(8, 1) = 3.661000;
	R0_terms(8, 2) = 5884.927000;
	R0_terms(9, 0) = 346.000000;
	R0_terms(9, 1) = 0.964000;
	R0_terms(9, 2) = 5507.553000;
	R0_terms(10, 0) = 329.000000;
	R0_terms(10, 1) = 5.900000;
	R0_terms(10, 2) = 5223.694000;
	R0_terms(11, 0) = 307.000000;
	R0_terms(11, 1) = 0.299000;
	R0_terms(11, 2) = 5573.143000;
	R0_terms(12, 0) = 243.000000;
	R0_terms(12, 1) = 4.273000;
	R0_terms(12, 2) = 11790.629000;
	R0_terms(13, 0) = 212.000000;
	R0_terms(13, 1) = 5.847000;
	R0_terms(13, 2) = 1577.344000;
	R0_terms(14, 0) = 186.000000;
	R0_terms(14, 1) = 5.022000;
	R0_terms(14, 2) = 10977.079000;
	R0_terms(15, 0) = 175.000000;
	R0_terms(15, 1) = 3.012000;
	R0_terms(15, 2) = 18849.228000;
	R0_terms(16, 0) = 110.000000;
	R0_terms(16, 1) = 5.055000;
	R0_terms(16, 2) = 5486.778000;
	R0_terms(17, 0) = 98.000000;
	R0_terms(17, 1) = 0.890000;
	R0_terms(17, 2) = 6069.780000;
	R0_terms(18, 0) = 86.000000;
	R0_terms(18, 1) = 5.690000;
	R0_terms(18, 2) = 15720.840000;
	R0_terms(19, 0) = 86.000000;
	R0_terms(19, 1) = 1.270000;
	R0_terms(19, 2) = 161000.690000;
	R0_terms(20, 0) = 85.000000;
	R0_terms(20, 1) = 0.270000;
	R0_terms(20, 2) = 17260.150000;
	R0_terms(21, 0) = 63.000000;
	R0_terms(21, 1) = 0.920000;
	R0_terms(21, 2) = 529.690000;
	R0_terms(22, 0) = 57.000000;
	R0_terms(22, 1) = 2.010000;
	R0_terms(22, 2) = 83996.850000;
	R0_terms(23, 0) = 56.000000;
	R0_terms(23, 1) = 5.240000;
	R0_terms(23, 2) = 71430.700000;
	R0_terms(24, 0) = 49.000000;
	R0_terms(24, 1) = 3.250000;
	R0_terms(24, 2) = 2544.310000;
	R0_terms(25, 0) = 47.000000;
	R0_terms(25, 1) = 2.580000;
	R0_terms(25, 2) = 775.520000;
	R0_terms(26, 0) = 45.000000;
	R0_terms(26, 1) = 5.540000;
	R0_terms(26, 2) = 9437.760000;
	R0_terms(27, 0) = 43.000000;
	R0_terms(27, 1) = 6.010000;
	R0_terms(27, 2) = 6275.960000;
	R0_terms(28, 0) = 39.000000;
	R0_terms(28, 1) = 5.360000;
	R0_terms(28, 2) = 4694.000000;
	R0_terms(29, 0) = 38.000000;
	R0_terms(29, 1) = 2.390000;
	R0_terms(29, 2) = 8827.390000;
	R0_terms(30, 0) = 37.000000;
	R0_terms(30, 1) = 0.830000;
	R0_terms(30, 2) = 19651.050000;
	R0_terms(31, 0) = 37.000000;
	R0_terms(31, 1) = 4.900000;
	R0_terms(31, 2) = 12139.550000;
	R0_terms(32, 0) = 36.000000;
	R0_terms(32, 1) = 1.670000;
	R0_terms(32, 2) = 12036.460000;
	R0_terms(33, 0) = 35.000000;
	R0_terms(33, 1) = 1.840000;
	R0_terms(33, 2) = 2942.460000;
	R0_terms(34, 0) = 33.000000;
	R0_terms(34, 1) = 0.240000;
	R0_terms(34, 2) = 7084.900000;
	R0_terms(35, 0) = 32.000000;
	R0_terms(35, 1) = 0.180000;
	R0_terms(35, 2) = 5088.630000;
	R0_terms(36, 0) = 32.000000;
	R0_terms(36, 1) = 1.780000;
	R0_terms(36, 2) = 398.150000;
	R0_terms(37, 0) = 28.000000;
	R0_terms(37, 1) = 1.210000;
	R0_terms(37, 2) = 6286.600000;
	R0_terms(38, 0) = 28.000000;
	R0_terms(38, 1) = 1.900000;
	R0_terms(38, 2) = 6279.550000;
	R0_terms(39, 0) = 26.000000;
	R0_terms(39, 1) = 4.590000;
	R0_terms(39, 2) = 10447.390000;

	arma::mat R1_terms(10, 3);
	R1_terms(0, 0) = 103019.000000;
	R1_terms(0, 1) = 1.107490;
	R1_terms(0, 2) = 6283.075850;
	R1_terms(1, 0) = 1721.000000;
	R1_terms(1, 1) = 1.064400;
	R1_terms(1, 2) = 12566.151700;
	R1_terms(2, 0) = 702.000000;
	R1_terms(2, 1) = 3.142000;
	R1_terms(2, 2) = 0.000000;
	R1_terms(3, 0) = 32.000000;
	R1_terms(3, 1) = 1.020000;
	R1_terms(3, 2) = 18849.230000;
	R1_terms(4, 0) = 31.000000;
	R1_terms(4, 1) = 2.840000;
	R1_terms(4, 2) = 5507.550000;
	R1_terms(5, 0) = 25.000000;
	R1_terms(5, 1) = 1.320000;
	R1_terms(5, 2) = 5223.690000;
	R1_terms(6, 0) = 18.000000;
	R1_terms(6, 1) = 1.420000;
	R1_terms(6, 2) = 1577.340000;
	R1_terms(7, 0) = 10.000000;
	R1_terms(7, 1) = 5.910000;
	R1_terms(7, 2) = 10977.080000;
	R1_terms(8, 0) = 9.000000;
	R1_terms(8, 1) = 1.420000;
	R1_terms(8, 2) = 6275.960000;
	R1_terms(9, 0) = 9.000000;
	R1_terms(9, 1) = 0.270000;
	R1_terms(9, 2) = 5486.780000;

	arma::mat R2_terms(6, 3);
	R2_terms(0, 0) = 4359.000000;
	R2_terms(0, 1) = 5.784600;
	R2_terms(0, 2) = 6283.075800;
	R2_terms(1, 0) = 124.000000;
	R2_terms(1, 1) = 5.579000;
	R2_terms(1, 2) = 12566.152000;
	R2_terms(2, 0) = 12.000000;
	R2_terms(2, 1) = 3.140000;
	R2_terms(2, 2) = 0.000000;
	R2_terms(3, 0) = 9.000000;
	R2_terms(3, 1) = 3.630000;
	R2_terms(3, 2) = 77713.770000;
	R2_terms(4, 0) = 6.000000;
	R2_terms(4, 1) = 1.870000;
	R2_terms(4, 2) = 5573.140000;
	R2_terms(5, 0) = 3.000000;
	R2_terms(5, 1) = 5.470000;
	R2_terms(5, 2) = 18849.000000;

	arma::mat R3_terms(2, 3);
	R3_terms(0, 0) = 145.000000;
	R3_terms(0, 1) = 4.273000;
	R3_terms(0, 2) = 6283.076000;
	R3_terms(1, 0) = 7.000000;
	R3_terms(1, 1) = 3.920000;
	R3_terms(1, 2) = 12566.150000;

	arma::mat R4_terms(1, 3);
	R4_terms(0, 0) = 4.000000;
	R4_terms(0, 1) = 2.560000;
	R4_terms(0, 2) = 6283.080000;

	A0.clear();
	B0.clear();
	C0.clear();

	A1.clear();
	B1.clear();
	C1.clear();

	A2.clear();
	B2.clear();
	C2.clear();

	A3.clear();
	B3.clear();
	C3.clear();

	A4.clear();
	B4.clear();
	C4.clear();

	A0 = R0_terms.col(0);
	B0 = R0_terms.col(1);
	C0 = R0_terms.col(2);

	A1 = R1_terms.col(0);
	B1 = R1_terms.col(1);
	C1 = R1_terms.col(2);

	A2 = R2_terms.col(0);
	B2 = R2_terms.col(1);
	C2 = R2_terms.col(2);

	A3 = R3_terms.col(0);
	B3 = R3_terms.col(1);
	C3 = R3_terms.col(2);

	A4 = R4_terms.col(0);
	B4 = R4_terms.col(1);
	C4 = R4_terms.col(2);

	// Compute the Earth heliocentric radius vector
	L0 = arma::sum(arma::sum(A0 % arma::cos(B0 + (C0 * JME))));
	L1 = arma::sum(arma::sum(A1 % arma::cos(B1 + (C1 * JME))));
	L2 = arma::sum(arma::sum(A2 % arma::cos(B2 + (C2 * JME))));
	L3 = arma::sum(arma::sum(A3 % arma::cos(B3 + (C3 * JME))));
	arma::mat L4_mat = A4 % arma::cos(B4 + (C4 * JME));
	L4 = L4_mat(0,0);

	// Units are in AU
	earth_heliocentric_position_radius = (L0 + (L1 * JME) + (L2 * JME*JME) + (L3 * JME*JME*JME) + (L4 * JME*JME*JME*JME)) / 1e8;
}

void Sun::sun_geocentric_position_calculation()
{
	// This function compute the sun position relative to the earth.

	sun_geocentric_position_longitude = earth_heliocentric_position_longitude + 180;
	// Limit the range to [0,360];
	sun_geocentric_position_longitude = fmod(sun_geocentric_position_longitude, 360.0);
	if (sun_geocentric_position_longitude<0)
	{
		sun_geocentric_position_longitude = sun_geocentric_position_longitude+360.0;
	}

	sun_geocentric_position_latitude = -earth_heliocentric_position_latitude;
	// Limit the range to [0,360]
	sun_geocentric_position_latitude = fmod(sun_geocentric_position_latitude, 360.0);
	if (sun_geocentric_position_longitude<0)
	{
		sun_geocentric_position_latitude = sun_geocentric_position_latitude+360.0;
	}
}

void Sun::nutation_calculation()
{
	// This function compute the nutation in longtitude and in obliquity, in
	// degrees.

	// All Xi are in degrees.
	double JCE = julian_ephemeris_century;

	// 1. Mean elongation of the moon from the sun
	double p1 = (1/189474);
	double p2 = -0.0019142;
	double p3 = 445267.11148;
	double p4 = 297.85036;
	double X0 = p1 * JCE*JCE*JCE + p2 * JCE*JCE  + p3 * JCE + p4;

	// 2. Mean anomaly of the sun (earth)
	p1 = -(1/300000);
	p2 = -0.0001603;
	p3 = 35999.05034;
	p4 = 357.52772;
	double X1 = p1 * JCE*JCE*JCE + p2 * JCE*JCE  + p3 * JCE + p4;

	// 3. Mean anomaly of the moon
	p1 = (1/56250);
	p2 = 0.0086972;
	p3 = 477198.867398;
	p4 = 134.96298;
	double X2 = p1 * JCE*JCE*JCE + p2 * JCE*JCE  + p3 * JCE + p4;

	// 4. Moon argument of latitude
	p1 = (1/327270);
	p2 = -0.0036825;
	p3 = 483202.017538;
	p4 = 93.27191;
	double X3 = p1 * JCE*JCE*JCE + p2 * JCE*JCE  + p3 * JCE + p4;

	// 5. Longitude of the ascending node of the moon's mean orbit on the
	// ecliptic, measured from the mean equinox of the date
	p1 = (1/450000);
	p2 = 0.0020708;
	p3 = -1934.136261;
	p4 = 125.04452;
	double X4 = p1 * JCE*JCE*JCE + p2 * JCE*JCE  + p3 * JCE + p4;

	arma::mat Y_terms(63, 5);
	Y_terms(0, 0) = 0.000000;
	Y_terms(0, 1) = 0.000000;
	Y_terms(0, 2) = 0.000000;
	Y_terms(0, 3) = 0.000000;
	Y_terms(0, 4) = 1.000000;
	Y_terms(1, 0) = -2.000000;
	Y_terms(1, 1) = 0.000000;
	Y_terms(1, 2) = 0.000000;
	Y_terms(1, 3) = 2.000000;
	Y_terms(1, 4) = 2.000000;
	Y_terms(2, 0) = 0.000000;
	Y_terms(2, 1) = 0.000000;
	Y_terms(2, 2) = 0.000000;
	Y_terms(2, 3) = 2.000000;
	Y_terms(2, 4) = 2.000000;
	Y_terms(3, 0) = 0.000000;
	Y_terms(3, 1) = 0.000000;
	Y_terms(3, 2) = 0.000000;
	Y_terms(3, 3) = 0.000000;
	Y_terms(3, 4) = 2.000000;
	Y_terms(4, 0) = 0.000000;
	Y_terms(4, 1) = 1.000000;
	Y_terms(4, 2) = 0.000000;
	Y_terms(4, 3) = 0.000000;
	Y_terms(4, 4) = 0.000000;
	Y_terms(5, 0) = 0.000000;
	Y_terms(5, 1) = 0.000000;
	Y_terms(5, 2) = 1.000000;
	Y_terms(5, 3) = 0.000000;
	Y_terms(5, 4) = 0.000000;
	Y_terms(6, 0) = -2.000000;
	Y_terms(6, 1) = 1.000000;
	Y_terms(6, 2) = 0.000000;
	Y_terms(6, 3) = 2.000000;
	Y_terms(6, 4) = 2.000000;
	Y_terms(7, 0) = 0.000000;
	Y_terms(7, 1) = 0.000000;
	Y_terms(7, 2) = 0.000000;
	Y_terms(7, 3) = 2.000000;
	Y_terms(7, 4) = 1.000000;
	Y_terms(8, 0) = 0.000000;
	Y_terms(8, 1) = 0.000000;
	Y_terms(8, 2) = 1.000000;
	Y_terms(8, 3) = 2.000000;
	Y_terms(8, 4) = 2.000000;
	Y_terms(9, 0) = -2.000000;
	Y_terms(9, 1) = -1.000000;
	Y_terms(9, 2) = 0.000000;
	Y_terms(9, 3) = 2.000000;
	Y_terms(9, 4) = 2.000000;
	Y_terms(10, 0) = -2.000000;
	Y_terms(10, 1) = 0.000000;
	Y_terms(10, 2) = 1.000000;
	Y_terms(10, 3) = 0.000000;
	Y_terms(10, 4) = 0.000000;
	Y_terms(11, 0) = -2.000000;
	Y_terms(11, 1) = 0.000000;
	Y_terms(11, 2) = 0.000000;
	Y_terms(11, 3) = 2.000000;
	Y_terms(11, 4) = 1.000000;
	Y_terms(12, 0) = 0.000000;
	Y_terms(12, 1) = 0.000000;
	Y_terms(12, 2) = -1.000000;
	Y_terms(12, 3) = 2.000000;
	Y_terms(12, 4) = 2.000000;
	Y_terms(13, 0) = 2.000000;
	Y_terms(13, 1) = 0.000000;
	Y_terms(13, 2) = 0.000000;
	Y_terms(13, 3) = 0.000000;
	Y_terms(13, 4) = 0.000000;
	Y_terms(14, 0) = 0.000000;
	Y_terms(14, 1) = 0.000000;
	Y_terms(14, 2) = 1.000000;
	Y_terms(14, 3) = 0.000000;
	Y_terms(14, 4) = 1.000000;
	Y_terms(15, 0) = 2.000000;
	Y_terms(15, 1) = 0.000000;
	Y_terms(15, 2) = -1.000000;
	Y_terms(15, 3) = 2.000000;
	Y_terms(15, 4) = 2.000000;
	Y_terms(16, 0) = 0.000000;
	Y_terms(16, 1) = 0.000000;
	Y_terms(16, 2) = -1.000000;
	Y_terms(16, 3) = 0.000000;
	Y_terms(16, 4) = 1.000000;
	Y_terms(17, 0) = 0.000000;
	Y_terms(17, 1) = 0.000000;
	Y_terms(17, 2) = 1.000000;
	Y_terms(17, 3) = 2.000000;
	Y_terms(17, 4) = 1.000000;
	Y_terms(18, 0) = -2.000000;
	Y_terms(18, 1) = 0.000000;
	Y_terms(18, 2) = 2.000000;
	Y_terms(18, 3) = 0.000000;
	Y_terms(18, 4) = 0.000000;
	Y_terms(19, 0) = 0.000000;
	Y_terms(19, 1) = 0.000000;
	Y_terms(19, 2) = -2.000000;
	Y_terms(19, 3) = 2.000000;
	Y_terms(19, 4) = 1.000000;
	Y_terms(20, 0) = 2.000000;
	Y_terms(20, 1) = 0.000000;
	Y_terms(20, 2) = 0.000000;
	Y_terms(20, 3) = 2.000000;
	Y_terms(20, 4) = 2.000000;
	Y_terms(21, 0) = 0.000000;
	Y_terms(21, 1) = 0.000000;
	Y_terms(21, 2) = 2.000000;
	Y_terms(21, 3) = 2.000000;
	Y_terms(21, 4) = 2.000000;
	Y_terms(22, 0) = 0.000000;
	Y_terms(22, 1) = 0.000000;
	Y_terms(22, 2) = 2.000000;
	Y_terms(22, 3) = 0.000000;
	Y_terms(22, 4) = 0.000000;
	Y_terms(23, 0) = -2.000000;
	Y_terms(23, 1) = 0.000000;
	Y_terms(23, 2) = 1.000000;
	Y_terms(23, 3) = 2.000000;
	Y_terms(23, 4) = 2.000000;
	Y_terms(24, 0) = 0.000000;
	Y_terms(24, 1) = 0.000000;
	Y_terms(24, 2) = 0.000000;
	Y_terms(24, 3) = 2.000000;
	Y_terms(24, 4) = 0.000000;
	Y_terms(25, 0) = -2.000000;
	Y_terms(25, 1) = 0.000000;
	Y_terms(25, 2) = 0.000000;
	Y_terms(25, 3) = 2.000000;
	Y_terms(25, 4) = 0.000000;
	Y_terms(26, 0) = 0.000000;
	Y_terms(26, 1) = 0.000000;
	Y_terms(26, 2) = -1.000000;
	Y_terms(26, 3) = 2.000000;
	Y_terms(26, 4) = 1.000000;
	Y_terms(27, 0) = 0.000000;
	Y_terms(27, 1) = 2.000000;
	Y_terms(27, 2) = 0.000000;
	Y_terms(27, 3) = 0.000000;
	Y_terms(27, 4) = 0.000000;
	Y_terms(28, 0) = 2.000000;
	Y_terms(28, 1) = 0.000000;
	Y_terms(28, 2) = -1.000000;
	Y_terms(28, 3) = 0.000000;
	Y_terms(28, 4) = 1.000000;
	Y_terms(29, 0) = -2.000000;
	Y_terms(29, 1) = 2.000000;
	Y_terms(29, 2) = 0.000000;
	Y_terms(29, 3) = 2.000000;
	Y_terms(29, 4) = 2.000000;
	Y_terms(30, 0) = 0.000000;
	Y_terms(30, 1) = 1.000000;
	Y_terms(30, 2) = 0.000000;
	Y_terms(30, 3) = 0.000000;
	Y_terms(30, 4) = 1.000000;
	Y_terms(31, 0) = -2.000000;
	Y_terms(31, 1) = 0.000000;
	Y_terms(31, 2) = 1.000000;
	Y_terms(31, 3) = 0.000000;
	Y_terms(31, 4) = 1.000000;
	Y_terms(32, 0) = 0.000000;
	Y_terms(32, 1) = -1.000000;
	Y_terms(32, 2) = 0.000000;
	Y_terms(32, 3) = 0.000000;
	Y_terms(32, 4) = 1.000000;
	Y_terms(33, 0) = 0.000000;
	Y_terms(33, 1) = 0.000000;
	Y_terms(33, 2) = 2.000000;
	Y_terms(33, 3) = -2.000000;
	Y_terms(33, 4) = 0.000000;
	Y_terms(34, 0) = 2.000000;
	Y_terms(34, 1) = 0.000000;
	Y_terms(34, 2) = -1.000000;
	Y_terms(34, 3) = 2.000000;
	Y_terms(34, 4) = 1.000000;
	Y_terms(35, 0) = 2.000000;
	Y_terms(35, 1) = 0.000000;
	Y_terms(35, 2) = 1.000000;
	Y_terms(35, 3) = 2.000000;
	Y_terms(35, 4) = 2.000000;
	Y_terms(36, 0) = 0.000000;
	Y_terms(36, 1) = 1.000000;
	Y_terms(36, 2) = 0.000000;
	Y_terms(36, 3) = 2.000000;
	Y_terms(36, 4) = 2.000000;
	Y_terms(37, 0) = -2.000000;
	Y_terms(37, 1) = 1.000000;
	Y_terms(37, 2) = 1.000000;
	Y_terms(37, 3) = 0.000000;
	Y_terms(37, 4) = 0.000000;
	Y_terms(38, 0) = 0.000000;
	Y_terms(38, 1) = -1.000000;
	Y_terms(38, 2) = 0.000000;
	Y_terms(38, 3) = 2.000000;
	Y_terms(38, 4) = 2.000000;
	Y_terms(39, 0) = 2.000000;
	Y_terms(39, 1) = 0.000000;
	Y_terms(39, 2) = 0.000000;
	Y_terms(39, 3) = 2.000000;
	Y_terms(39, 4) = 1.000000;
	Y_terms(40, 0) = 2.000000;
	Y_terms(40, 1) = 0.000000;
	Y_terms(40, 2) = 1.000000;
	Y_terms(40, 3) = 0.000000;
	Y_terms(40, 4) = 0.000000;
	Y_terms(41, 0) = -2.000000;
	Y_terms(41, 1) = 0.000000;
	Y_terms(41, 2) = 2.000000;
	Y_terms(41, 3) = 2.000000;
	Y_terms(41, 4) = 2.000000;
	Y_terms(42, 0) = -2.000000;
	Y_terms(42, 1) = 0.000000;
	Y_terms(42, 2) = 1.000000;
	Y_terms(42, 3) = 2.000000;
	Y_terms(42, 4) = 1.000000;
	Y_terms(43, 0) = 2.000000;
	Y_terms(43, 1) = 0.000000;
	Y_terms(43, 2) = -2.000000;
	Y_terms(43, 3) = 0.000000;
	Y_terms(43, 4) = 1.000000;
	Y_terms(44, 0) = 2.000000;
	Y_terms(44, 1) = 0.000000;
	Y_terms(44, 2) = 0.000000;
	Y_terms(44, 3) = 0.000000;
	Y_terms(44, 4) = 1.000000;
	Y_terms(45, 0) = 0.000000;
	Y_terms(45, 1) = -1.000000;
	Y_terms(45, 2) = 1.000000;
	Y_terms(45, 3) = 0.000000;
	Y_terms(45, 4) = 0.000000;
	Y_terms(46, 0) = -2.000000;
	Y_terms(46, 1) = -1.000000;
	Y_terms(46, 2) = 0.000000;
	Y_terms(46, 3) = 2.000000;
	Y_terms(46, 4) = 1.000000;
	Y_terms(47, 0) = -2.000000;
	Y_terms(47, 1) = 0.000000;
	Y_terms(47, 2) = 0.000000;
	Y_terms(47, 3) = 0.000000;
	Y_terms(47, 4) = 1.000000;
	Y_terms(48, 0) = 0.000000;
	Y_terms(48, 1) = 0.000000;
	Y_terms(48, 2) = 2.000000;
	Y_terms(48, 3) = 2.000000;
	Y_terms(48, 4) = 1.000000;
	Y_terms(49, 0) = -2.000000;
	Y_terms(49, 1) = 0.000000;
	Y_terms(49, 2) = 2.000000;
	Y_terms(49, 3) = 0.000000;
	Y_terms(49, 4) = 1.000000;
	Y_terms(50, 0) = -2.000000;
	Y_terms(50, 1) = 1.000000;
	Y_terms(50, 2) = 0.000000;
	Y_terms(50, 3) = 2.000000;
	Y_terms(50, 4) = 1.000000;
	Y_terms(51, 0) = 0.000000;
	Y_terms(51, 1) = 0.000000;
	Y_terms(51, 2) = 1.000000;
	Y_terms(51, 3) = -2.000000;
	Y_terms(51, 4) = 0.000000;
	Y_terms(52, 0) = -1.000000;
	Y_terms(52, 1) = 0.000000;
	Y_terms(52, 2) = 1.000000;
	Y_terms(52, 3) = 0.000000;
	Y_terms(52, 4) = 0.000000;
	Y_terms(53, 0) = -2.000000;
	Y_terms(53, 1) = 1.000000;
	Y_terms(53, 2) = 0.000000;
	Y_terms(53, 3) = 0.000000;
	Y_terms(53, 4) = 0.000000;
	Y_terms(54, 0) = 1.000000;
	Y_terms(54, 1) = 0.000000;
	Y_terms(54, 2) = 0.000000;
	Y_terms(54, 3) = 0.000000;
	Y_terms(54, 4) = 0.000000;
	Y_terms(55, 0) = 0.000000;
	Y_terms(55, 1) = 0.000000;
	Y_terms(55, 2) = 1.000000;
	Y_terms(55, 3) = 2.000000;
	Y_terms(55, 4) = 0.000000;
	Y_terms(56, 0) = 0.000000;
	Y_terms(56, 1) = 0.000000;
	Y_terms(56, 2) = -2.000000;
	Y_terms(56, 3) = 2.000000;
	Y_terms(56, 4) = 2.000000;
	Y_terms(57, 0) = -1.000000;
	Y_terms(57, 1) = -1.000000;
	Y_terms(57, 2) = 1.000000;
	Y_terms(57, 3) = 0.000000;
	Y_terms(57, 4) = 0.000000;
	Y_terms(58, 0) = 0.000000;
	Y_terms(58, 1) = 1.000000;
	Y_terms(58, 2) = 1.000000;
	Y_terms(58, 3) = 0.000000;
	Y_terms(58, 4) = 0.000000;
	Y_terms(59, 0) = 0.000000;
	Y_terms(59, 1) = -1.000000;
	Y_terms(59, 2) = 1.000000;
	Y_terms(59, 3) = 2.000000;
	Y_terms(59, 4) = 2.000000;
	Y_terms(60, 0) = 2.000000;
	Y_terms(60, 1) = -1.000000;
	Y_terms(60, 2) = -1.000000;
	Y_terms(60, 3) = 2.000000;
	Y_terms(60, 4) = 2.000000;
	Y_terms(61, 0) = 0.000000;
	Y_terms(61, 1) = 0.000000;
	Y_terms(61, 2) = 3.000000;
	Y_terms(61, 3) = 2.000000;
	Y_terms(61, 4) = 2.000000;
	Y_terms(62, 0) = 2.000000;
	Y_terms(62, 1) = -1.000000;
	Y_terms(62, 2) = 0.000000;
	Y_terms(62, 3) = 2.000000;
	Y_terms(62, 4) = 2.000000;

	arma::mat nutation_terms(63, 4);
	nutation_terms(0, 0) = -171996.000000;
	nutation_terms(0, 1) = -174.200000;
	nutation_terms(0, 2) = 92025.000000;
	nutation_terms(0, 3) = 8.900000;
	nutation_terms(1, 0) = -13187.000000;
	nutation_terms(1, 1) = -1.600000;
	nutation_terms(1, 2) = 5736.000000;
	nutation_terms(1, 3) = -3.100000;
	nutation_terms(2, 0) = -2274.000000;
	nutation_terms(2, 1) = -0.200000;
	nutation_terms(2, 2) = 977.000000;
	nutation_terms(2, 3) = -0.500000;
	nutation_terms(3, 0) = 2062.000000;
	nutation_terms(3, 1) = 0.200000;
	nutation_terms(3, 2) = -895.000000;
	nutation_terms(3, 3) = 0.500000;
	nutation_terms(4, 0) = 1426.000000;
	nutation_terms(4, 1) = -3.400000;
	nutation_terms(4, 2) = 54.000000;
	nutation_terms(4, 3) = -0.100000;
	nutation_terms(5, 0) = 712.000000;
	nutation_terms(5, 1) = 0.100000;
	nutation_terms(5, 2) = -7.000000;
	nutation_terms(5, 3) = 0.000000;
	nutation_terms(6, 0) = -517.000000;
	nutation_terms(6, 1) = 1.200000;
	nutation_terms(6, 2) = 224.000000;
	nutation_terms(6, 3) = -0.600000;
	nutation_terms(7, 0) = -386.000000;
	nutation_terms(7, 1) = -0.400000;
	nutation_terms(7, 2) = 200.000000;
	nutation_terms(7, 3) = 0.000000;
	nutation_terms(8, 0) = -301.000000;
	nutation_terms(8, 1) = 0.000000;
	nutation_terms(8, 2) = 129.000000;
	nutation_terms(8, 3) = -0.100000;
	nutation_terms(9, 0) = 217.000000;
	nutation_terms(9, 1) = -0.500000;
	nutation_terms(9, 2) = -95.000000;
	nutation_terms(9, 3) = 0.300000;
	nutation_terms(10, 0) = -158.000000;
	nutation_terms(10, 1) = 0.000000;
	nutation_terms(10, 2) = 0.000000;
	nutation_terms(10, 3) = 0.000000;
	nutation_terms(11, 0) = 129.000000;
	nutation_terms(11, 1) = 0.100000;
	nutation_terms(11, 2) = -70.000000;
	nutation_terms(11, 3) = 0.000000;
	nutation_terms(12, 0) = 123.000000;
	nutation_terms(12, 1) = 0.000000;
	nutation_terms(12, 2) = -53.000000;
	nutation_terms(12, 3) = 0.000000;
	nutation_terms(13, 0) = 63.000000;
	nutation_terms(13, 1) = 0.000000;
	nutation_terms(13, 2) = 0.000000;
	nutation_terms(13, 3) = 0.000000;
	nutation_terms(14, 0) = 63.000000;
	nutation_terms(14, 1) = 0.100000;
	nutation_terms(14, 2) = -33.000000;
	nutation_terms(14, 3) = 0.000000;
	nutation_terms(15, 0) = -59.000000;
	nutation_terms(15, 1) = 0.000000;
	nutation_terms(15, 2) = 26.000000;
	nutation_terms(15, 3) = 0.000000;
	nutation_terms(16, 0) = -58.000000;
	nutation_terms(16, 1) = -0.100000;
	nutation_terms(16, 2) = 32.000000;
	nutation_terms(16, 3) = 0.000000;
	nutation_terms(17, 0) = -51.000000;
	nutation_terms(17, 1) = 0.000000;
	nutation_terms(17, 2) = 27.000000;
	nutation_terms(17, 3) = 0.000000;
	nutation_terms(18, 0) = 48.000000;
	nutation_terms(18, 1) = 0.000000;
	nutation_terms(18, 2) = 0.000000;
	nutation_terms(18, 3) = 0.000000;
	nutation_terms(19, 0) = 46.000000;
	nutation_terms(19, 1) = 0.000000;
	nutation_terms(19, 2) = -24.000000;
	nutation_terms(19, 3) = 0.000000;
	nutation_terms(20, 0) = -38.000000;
	nutation_terms(20, 1) = 0.000000;
	nutation_terms(20, 2) = 16.000000;
	nutation_terms(20, 3) = 0.000000;
	nutation_terms(21, 0) = -31.000000;
	nutation_terms(21, 1) = 0.000000;
	nutation_terms(21, 2) = 13.000000;
	nutation_terms(21, 3) = 0.000000;
	nutation_terms(22, 0) = 29.000000;
	nutation_terms(22, 1) = 0.000000;
	nutation_terms(22, 2) = 0.000000;
	nutation_terms(22, 3) = 0.000000;
	nutation_terms(23, 0) = 29.000000;
	nutation_terms(23, 1) = 0.000000;
	nutation_terms(23, 2) = -12.000000;
	nutation_terms(23, 3) = 0.000000;
	nutation_terms(24, 0) = 26.000000;
	nutation_terms(24, 1) = 0.000000;
	nutation_terms(24, 2) = 0.000000;
	nutation_terms(24, 3) = 0.000000;
	nutation_terms(25, 0) = -22.000000;
	nutation_terms(25, 1) = 0.000000;
	nutation_terms(25, 2) = 0.000000;
	nutation_terms(25, 3) = 0.000000;
	nutation_terms(26, 0) = 21.000000;
	nutation_terms(26, 1) = 0.000000;
	nutation_terms(26, 2) = -10.000000;
	nutation_terms(26, 3) = 0.000000;
	nutation_terms(27, 0) = 17.000000;
	nutation_terms(27, 1) = -0.100000;
	nutation_terms(27, 2) = 0.000000;
	nutation_terms(27, 3) = 0.000000;
	nutation_terms(28, 0) = 16.000000;
	nutation_terms(28, 1) = 0.000000;
	nutation_terms(28, 2) = -8.000000;
	nutation_terms(28, 3) = 0.000000;
	nutation_terms(29, 0) = -16.000000;
	nutation_terms(29, 1) = 0.100000;
	nutation_terms(29, 2) = 7.000000;
	nutation_terms(29, 3) = 0.000000;
	nutation_terms(30, 0) = -15.000000;
	nutation_terms(30, 1) = 0.000000;
	nutation_terms(30, 2) = 9.000000;
	nutation_terms(30, 3) = 0.000000;
	nutation_terms(31, 0) = -13.000000;
	nutation_terms(31, 1) = 0.000000;
	nutation_terms(31, 2) = 7.000000;
	nutation_terms(31, 3) = 0.000000;
	nutation_terms(32, 0) = -12.000000;
	nutation_terms(32, 1) = 0.000000;
	nutation_terms(32, 2) = 6.000000;
	nutation_terms(32, 3) = 0.000000;
	nutation_terms(33, 0) = 11.000000;
	nutation_terms(33, 1) = 0.000000;
	nutation_terms(33, 2) = 0.000000;
	nutation_terms(33, 3) = 0.000000;
	nutation_terms(34, 0) = -10.000000;
	nutation_terms(34, 1) = 0.000000;
	nutation_terms(34, 2) = 5.000000;
	nutation_terms(34, 3) = 0.000000;
	nutation_terms(35, 0) = -8.000000;
	nutation_terms(35, 1) = 0.000000;
	nutation_terms(35, 2) = 3.000000;
	nutation_terms(35, 3) = 0.000000;
	nutation_terms(36, 0) = 7.000000;
	nutation_terms(36, 1) = 0.000000;
	nutation_terms(36, 2) = -3.000000;
	nutation_terms(36, 3) = 0.000000;
	nutation_terms(37, 0) = -7.000000;
	nutation_terms(37, 1) = 0.000000;
	nutation_terms(37, 2) = 0.000000;
	nutation_terms(37, 3) = 0.000000;
	nutation_terms(38, 0) = -7.000000;
	nutation_terms(38, 1) = 0.000000;
	nutation_terms(38, 2) = 3.000000;
	nutation_terms(38, 3) = 0.000000;
	nutation_terms(39, 0) = -7.000000;
	nutation_terms(39, 1) = 0.000000;
	nutation_terms(39, 2) = 3.000000;
	nutation_terms(39, 3) = 0.000000;
	nutation_terms(40, 0) = 6.000000;
	nutation_terms(40, 1) = 0.000000;
	nutation_terms(40, 2) = 0.000000;
	nutation_terms(40, 3) = 0.000000;
	nutation_terms(41, 0) = 6.000000;
	nutation_terms(41, 1) = 0.000000;
	nutation_terms(41, 2) = -3.000000;
	nutation_terms(41, 3) = 0.000000;
	nutation_terms(42, 0) = 6.000000;
	nutation_terms(42, 1) = 0.000000;
	nutation_terms(42, 2) = -3.000000;
	nutation_terms(42, 3) = 0.000000;
	nutation_terms(43, 0) = -6.000000;
	nutation_terms(43, 1) = 0.000000;
	nutation_terms(43, 2) = 3.000000;
	nutation_terms(43, 3) = 0.000000;
	nutation_terms(44, 0) = -6.000000;
	nutation_terms(44, 1) = 0.000000;
	nutation_terms(44, 2) = 3.000000;
	nutation_terms(44, 3) = 0.000000;
	nutation_terms(45, 0) = 5.000000;
	nutation_terms(45, 1) = 0.000000;
	nutation_terms(45, 2) = 0.000000;
	nutation_terms(45, 3) = 0.000000;
	nutation_terms(46, 0) = -5.000000;
	nutation_terms(46, 1) = 0.000000;
	nutation_terms(46, 2) = 3.000000;
	nutation_terms(46, 3) = 0.000000;
	nutation_terms(47, 0) = -5.000000;
	nutation_terms(47, 1) = 0.000000;
	nutation_terms(47, 2) = 3.000000;
	nutation_terms(47, 3) = 0.000000;
	nutation_terms(48, 0) = -5.000000;
	nutation_terms(48, 1) = 0.000000;
	nutation_terms(48, 2) = 3.000000;
	nutation_terms(48, 3) = 0.000000;
	nutation_terms(49, 0) = 4.000000;
	nutation_terms(49, 1) = 0.000000;
	nutation_terms(49, 2) = 0.000000;
	nutation_terms(49, 3) = 0.000000;
	nutation_terms(50, 0) = 4.000000;
	nutation_terms(50, 1) = 0.000000;
	nutation_terms(50, 2) = 0.000000;
	nutation_terms(50, 3) = 0.000000;
	nutation_terms(51, 0) = 4.000000;
	nutation_terms(51, 1) = 0.000000;
	nutation_terms(51, 2) = 0.000000;
	nutation_terms(51, 3) = 0.000000;
	nutation_terms(52, 0) = -4.000000;
	nutation_terms(52, 1) = 0.000000;
	nutation_terms(52, 2) = 0.000000;
	nutation_terms(52, 3) = 0.000000;
	nutation_terms(53, 0) = -4.000000;
	nutation_terms(53, 1) = 0.000000;
	nutation_terms(53, 2) = 0.000000;
	nutation_terms(53, 3) = 0.000000;
	nutation_terms(54, 0) = -4.000000;
	nutation_terms(54, 1) = 0.000000;
	nutation_terms(54, 2) = 0.000000;
	nutation_terms(54, 3) = 0.000000;
	nutation_terms(55, 0) = 3.000000;
	nutation_terms(55, 1) = 0.000000;
	nutation_terms(55, 2) = 0.000000;
	nutation_terms(55, 3) = 0.000000;
	nutation_terms(56, 0) = -3.000000;
	nutation_terms(56, 1) = 0.000000;
	nutation_terms(56, 2) = 0.000000;
	nutation_terms(56, 3) = 0.000000;
	nutation_terms(57, 0) = -3.000000;
	nutation_terms(57, 1) = 0.000000;
	nutation_terms(57, 2) = 0.000000;
	nutation_terms(57, 3) = 0.000000;
	nutation_terms(58, 0) = -3.000000;
	nutation_terms(58, 1) = 0.000000;
	nutation_terms(58, 2) = 0.000000;
	nutation_terms(58, 3) = 0.000000;
	nutation_terms(59, 0) = -3.000000;
	nutation_terms(59, 1) = 0.000000;
	nutation_terms(59, 2) = 0.000000;
	nutation_terms(59, 3) = 0.000000;
	nutation_terms(60, 0) = -3.000000;
	nutation_terms(60, 1) = 0.000000;
	nutation_terms(60, 2) = 0.000000;
	nutation_terms(60, 3) = 0.000000;
	nutation_terms(61, 0) = -3.000000;
	nutation_terms(61, 1) = 0.000000;
	nutation_terms(61, 2) = 0.000000;
	nutation_terms(61, 3) = 0.000000;
	nutation_terms(62, 0) = -3.000000;
	nutation_terms(62, 1) = 0.000000;
	nutation_terms(62, 2) = 0.000000;
	nutation_terms(62, 3) = 0.000000;

	arma::mat Xi(5, 1);
	Xi(0, 0) = X0;
	Xi(1, 0) = X1;
	Xi(2, 0) = X2;
	Xi(3, 0) = X3;
	Xi(4, 0) = X4;

	arma::mat tabulated_argument = (Y_terms * Xi) * pi/180;
	arma::mat delta_longitude = ((nutation_terms.col(0) + (nutation_terms.col(1) * JCE))) % arma::sin(tabulated_argument);
	arma::mat delta_obliquity = ((nutation_terms.col(2) + (nutation_terms.col(3) * JCE))) % arma::cos(tabulated_argument);

	// Nutation in longitude
	nutation_longitude = arma::sum(arma::sum(delta_longitude)) / 36000000;

	// Nutation in obliquity
	nutation_obliquity = arma::sum(arma::sum(delta_obliquity)) / 36000000;
}

void Sun::true_obliquity_calculation()
{
	// This function compute the true obliquity of the ecliptic.

	double p1 = 2.45;
	double p2 = 5.79;
	double p3 = 27.87;
	double p4 = 7.12;
	double p5 = -39.05;
	double p6 = -249.67;
	double p7 = -51.38;
	double p8 = 1999.25;
	double p9 = -1.55;
	double p10 = -4680.93;
	double p11 = 84381.448;

	double U = julian_ephemeris_millenium/10;
	double mean_obliquity = p1*pow(U,10.0) + p2*pow(U,9.0)+ p3*pow(U,8.0) + p4*pow(U,7.0) + p5*pow(U,6.0) + p6*pow(U,5.0) + p7*pow(U,4.0) + p8*pow(U,3.0) + p9*pow(U,2.0) + p10*U + p11;

	true_obliquity = (mean_obliquity/3600) + nutation_obliquity;
}

void Sun::abberation_correction_calculation()
{
	// This function compute the abberation_correction, as a function of the
	// earth-sun distance.

	abberation_correction = -20.4898/(3600*earth_heliocentric_position_radius);
}

void Sun::apparent_sun_longitude_calculation()
{
	// This function compute the sun apparent longitude

	apparent_sun_longitude = sun_geocentric_position_longitude + nutation_longitude + abberation_correction;
}

void Sun::apparent_stime_at_greenwich_calculation()
{
	// This function compute the apparent sideral time at Greenwich.

	double JD = julian_day;
	double JC = julian_century;

	// Mean sideral time, in degrees
	double mean_stime = 280.46061837 + (360.98564736629*(JD-2451545)) + (0.000387933*JC*JC) - (JC*JC*JC/38710000);

	// Limit the range to [0-360];
	mean_stime = fmod(mean_stime, 360.0);
	if(mean_stime<0)
	{
		mean_stime = mean_stime+360.0;
	}

	apparent_stime_at_greenwich = mean_stime + (nutation_longitude * cos(true_obliquity * pi/180));
}

void Sun::sun_rigth_ascension_calculation()
{
	// This function compute the sun rigth ascension.

	double argument_numerator = (sin(apparent_sun_longitude * pi/180) * cos(true_obliquity * pi/180)) - (tan(sun_geocentric_position_latitude * pi/180) * sin(true_obliquity * pi/180));
	double argument_denominator = cos(apparent_sun_longitude * pi/180);

	sun_rigth_ascension = atan2(argument_numerator, argument_denominator) * 180/pi;
	// Limit the range to [0,360];
	sun_rigth_ascension = fmod(sun_rigth_ascension, 360.0);
	if(sun_rigth_ascension<0)
	{
		sun_rigth_ascension = sun_rigth_ascension+360.0;
	}
}


void Sun::sun_geocentric_declination_calculation()
{
	double argument = (sin(sun_geocentric_position_latitude * pi/180) * cos(true_obliquity * pi/180)) + (cos(sun_geocentric_position_latitude * pi/180) * sin(true_obliquity * pi/180) * sin(apparent_sun_longitude * pi/180));
	sun_geocentric_declination = asin(argument) * 180/pi;
}

void Sun::observer_local_hour_calculation()
{
	observer_local_hour = apparent_stime_at_greenwich + location_longitude - sun_rigth_ascension;
	// Set the range to [0-360]
	observer_local_hour = fmod(observer_local_hour, 360.0);
	if(observer_local_hour<0)
	{
		observer_local_hour = observer_local_hour+360.0;
	}
}

void Sun::topocentric_sun_position_calculate()
{
	// This function compute the sun position (rigth ascension and declination)
	// with respect to the observer local position at the Earth surface.

	// Equatorial horizontal parallax of the sun in degrees
	double eq_horizontal_parallax = 8.794 / (3600 * earth_heliocentric_position_radius);

	// Term u, used in the following calculations (in radians)
	double u = atan(0.99664719 * tan(location_latitude * pi/180));

	// Term x, used in the following calculations
	double x = cos(u) + ((location_altitude/6378140) * cos(location_latitude * pi/180));

	// Term y, used in the following calculations
	double y = (0.99664719 * sin(u)) + ((location_altitude/6378140) * sin(location_latitude * pi/180));

	// Parallax in the sun rigth ascension (in radians)
	double nominator = -x * sin(eq_horizontal_parallax * pi/180) * sin(observer_local_hour * pi/180);
	double denominator = cos(sun_geocentric_declination * pi/180) - (x * sin(eq_horizontal_parallax * pi/180) * cos(observer_local_hour * pi/180));
	double sun_rigth_ascension_parallax = atan2(nominator, denominator);
	// Conversion to degrees.
	topocentric_sun_position_rigth_ascension_parallax = sun_rigth_ascension_parallax * 180/pi;

	// Topocentric sun rigth ascension (in degrees)
	topocentric_sun_position_rigth_ascension = sun_rigth_ascension + (sun_rigth_ascension_parallax * 180/pi);

	// Topocentric sun declination (in degrees)
	nominator = (sin(sun_geocentric_declination * pi/180) - (y*sin(eq_horizontal_parallax * pi/180))) * cos(sun_rigth_ascension_parallax);
	denominator = cos(sun_geocentric_declination * pi/180) - (x*sin(eq_horizontal_parallax * pi/180)) * cos(observer_local_hour * pi/180);
	topocentric_sun_position_declination = atan2(nominator, denominator) * 180/pi;
}

void Sun::topocentric_local_hour_calculate()
{
	// This function compute the topocentric local jour angle in degrees

	topocentric_local_hour = observer_local_hour - topocentric_sun_position_rigth_ascension_parallax;
}

void Sun::sun_topocentric_zenith_angle_calculate()
{
	// This function compute the sun zenith angle, taking into account the
	// atmospheric refraction. A default temperature of 283K and a
	// default pressure of 1010 mbar are used.

	// Topocentric elevation, without atmospheric refraction
	double argument = (sin(location_latitude * pi/180) * sin(topocentric_sun_position_declination * pi/180)) + (cos(location_latitude * pi/180) * cos(topocentric_sun_position_declination * pi/180) * cos(topocentric_local_hour * pi/180));
	double true_elevation = asin(argument) * 180/pi;

	// Atmospheric refraction correction (in degrees)
	argument = true_elevation + (10.3/(true_elevation + 5.11));
	double refraction_corr = 1.02 / (60 * tan(argument * pi/180));

	// For exact pressure and temperature correction, use this,
	// with P the pressure in mbar amd T the temperature in Kelvins:
	// refraction_corr = (P/1010) * (283/T) * 1.02 / (60 * tan(argument * pi/180));

	// Apparent elevation
	double apparent_elevation;
	if(true_elevation > -5)
	{
		apparent_elevation = true_elevation + refraction_corr;
	}
	else
	{
		apparent_elevation = true_elevation;
	}

	sun_zenith = 90 - apparent_elevation;

	// Topocentric azimuth angle. The +180 conversion is to pass from astronomer
	// notation (westward from south) to navigation notation (eastward from
	// north);
	double nominator = sin(topocentric_local_hour * pi/180);
	double denominator = (cos(topocentric_local_hour * pi/180) * sin(location_latitude * pi/180)) - (tan(topocentric_sun_position_declination * pi/180) * cos(location_latitude * pi/180));
	sun_azimuth = (atan2(nominator, denominator) * 180/pi) + 180;
	// Set the range to [0-360]
	sun_azimuth = fmod(sun_azimuth, 360.0);
	if (sun_azimuth<0)
	{
		sun_azimuth = sun_azimuth+360.0;
	}
}
