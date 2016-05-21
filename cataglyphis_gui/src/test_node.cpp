#include <ros/ros.h>

#include <QApplication>
#include "cataglyphis_gui.h"

int main(int argc, char **argv)
{
    Q_INIT_RESOURCE(resources);
    ros::init(argc, argv, "GUI_Node");
	ROS_INFO("GUI_Node - ros::init complete");
	ros::NodeHandle nh;
	ROS_INFO("GUI_Node - node handle created");
	
	QApplication qCoreApp(argc, argv);
    Cataglyphis_Gui cataglyphis_gui;
    cataglyphis_gui.show();

    return qCoreApp.exec();
    
	ros::Rate loopRate(50); //set loop rate to 50Hz

    ros::spinOnce();
    loopRate.sleep();

    
}
