#include <ros/ros.h>
#include <hw_interface/hw_interface.hpp>


int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
           ros::console::notifyLoggerLevelsChanged();
        }
    std::string node_type;
    if(ros::param::get("node_type", node_type)==false) node_type = "hw_interface";
    ROS_INFO("hw_interface Start");
    ros::init(argc, argv, node_type);
    ROS_INFO(" - ros::init complete");
    ros::NodeHandle nh;
    ROS_INFO(" - node handle created");

    hw_interface interfaces;
    ROS_DEBUG("HW_Interface Obj start");

    while(ros::ok())
    {
        ROS_INFO("Sitting In Main");
        ros::Duration pause(10);
        pause.sleep();
    }
    return 0;
}
