#include <ros/ros.h>

#include "cataglyphis_gui.h"
#include <stdio.h>
#include <iostream>
#include <string>
#include <QApplication>

//#include "../simulation/cataglyphis_simulation.h"


//messages::ActuatorOut actuator_msg;
//messages::ExecStateMachineInfo exec_msg;
//robot_control::Keys_Pressed keys_msg;
//messages::Servo_command servo_msg;

int main(int argc, char **argv)
{
    Q_INIT_RESOURCE(resources);

	ROS_INFO("Keyboard_Node - main() start");
	ros::init(argc, argv, "keyboard_node");
	ROS_INFO("Keyboard_Node - ros::init complete");
	ros::NodeHandle nh;
	ROS_INFO("Keyboard_Node - node handle created");
	
	QApplication a(argc, argv);
    Cataglyphis_Gui w;
    w.show();

    return a.exec();
    
	ros::Rate loopRate(50); //set loop rate to 50Hz

    ros::spinOnce();
    loopRate.sleep();

    
}
