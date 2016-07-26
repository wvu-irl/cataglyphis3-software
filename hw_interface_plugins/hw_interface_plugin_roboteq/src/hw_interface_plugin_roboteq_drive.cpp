#include <ros/ros.h>

#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq_drive.hpp>

void hw_interface_plugin_roboteq::roboteq_drive::rosMsgCallback(const messages::ActuatorOut::ConstPtr &msgIn)
{
    std::string motorSpeedCmds = "";

    if(roboteqType == controller_t::Right_Drive_Roboteq)
    {
        motorSpeedCmds += "!G 1 " + boost::lexical_cast<std::string>(msgIn->fr_speed_cmd) + "\r";
        motorSpeedCmds += "!G 2 " + boost::lexical_cast<std::string>(msgIn->mr_speed_cmd) + "\r";
        motorSpeedCmds += "!G 3 " + boost::lexical_cast<std::string>(msgIn->br_speed_cmd) + "\r";
        postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(motorSpeedCmds));
    }
    else if(roboteqType == controller_t::Left_Drive_Roboteq)
    {
        motorSpeedCmds += "!G 1 " + boost::lexical_cast<std::string>(msgIn->fl_speed_cmd) + "\r";
        motorSpeedCmds += "!G 2 " + boost::lexical_cast<std::string>(msgIn->ml_speed_cmd) + "\r";
        motorSpeedCmds += "!G 3 " + boost::lexical_cast<std::string>(msgIn->bl_speed_cmd) + "\r";
        postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(motorSpeedCmds));
    }
    else
    {
        ROS_WARN("%s:: No Data written because of Incorrect Roboteq Type", pluginName.c_str());
    }

    //need to add monitoring facilities to monitor health
}

bool hw_interface_plugin_roboteq::roboteq_drive::implInit()
{

    readLength = sizeof(roboteq_drive_packet::drive_packet_struct_t);
    
    headerString = roboteq_drive_packet::HEADER;
    footerString = roboteq_drive_packet::FOOTER;

    std::string tempString;
    if(ros::param::get(pluginName+"/subscribeToTopic", tempString))
    {
        rosDataSub = nh->subscribe(tempString, 1, &roboteq_drive::rosMsgCallback, this);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic subscription name", pluginName.c_str());
    }

    if(ros::param::get(pluginName+"/publishToTopic", tempString))
    {
        rosDataPub = nh->advertise<messages::encoder_data>(tempString, 1, false);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic advertisment name", pluginName.c_str());
    }

//need to start async timers here for grabber monitoring
    return true;
}

bool hw_interface_plugin_roboteq::roboteq_drive::implStart()
{

    return true;
}

bool hw_interface_plugin_roboteq::roboteq_drive::implStop()
{

    return true;
}

bool hw_interface_plugin_roboteq::roboteq_drive::implDataHandler(const long &length, int arrayStartPos)
{
    ROS_DEBUG("%s :: Roboteq Drive Implementation Data Handler", pluginName.c_str());

    //should check size of buffer is equal to size of msg, just in case.
    ROS_DEBUG("Buf Pointer: 0x%p\r\n", &receivedData[arrayStartPos]);
    std::printf("Contents: ");
    for(int i = 0; i < length; i++)
    {
        std::printf("%x | ", receivedData[arrayStartPos + i]);
    }
    std::printf("\r\n");
    uint32_t crcCheck = calcCRC32Block(&receivedData[arrayStartPos],
                                            roboteq_drive_packet::PACKET_SIZE_MINUS_CRC_AND_FOOTER);
    ROS_DEBUG("%s :: CRC32: %x", pluginName.c_str(), crcCheck);

    if(true /*verify checksum here*/)
    {
        roboteq_drive_packet::deserializeBufAndPublish(rosDataPub, &receivedData[arrayStartPos]);
    }
    else
    {
        ROS_ERROR("%s:: Invalid Checksum! Calculated: 0x%x, Actual: 0x%x", pluginName.c_str(), crcCheck, 00);
    }

    return true;
}
