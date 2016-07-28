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