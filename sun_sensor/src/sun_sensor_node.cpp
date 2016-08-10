#include <sun_sensor/sun_sensor.hpp>
#include <messages/SunSensorOut.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sun_sensor_node");
	ROS_INFO("sun_sensor_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(20);
	//ros::Publisher pubSunSensorOut = nh.advertise<messages::SunSensorOut>("/sunsensor/sunsensorout/sunsensorout",1);
	//messages::SunSensorOut msgSunSensorOut;

	SunSensor sunSensor;
	Sun sun;

	while(ros::ok())
	{
		//pubSunSensorOut.publish(msgSunSensorOut);
		loop_rate.sleep();
		ros::spinOnce();		
	}

	return 0;
}