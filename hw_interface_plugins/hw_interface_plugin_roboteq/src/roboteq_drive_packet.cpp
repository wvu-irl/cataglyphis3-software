#include <ros/ros.h>

#include <hw_interface_plugin_roboteq/roboteq_drive_packet.hpp>


messages::encoder_data
    hw_interface_plugin_roboteq::roboteq_drive_packet::deserializeBufIntoMsg(void *buffer)
{
    messages::encoder_data msg;

    msg.counter					=   ((drive_packet_struct_t*)buffer)->counter^0xff;
    msg.clock					=   ((drive_packet_struct_t*)buffer)->clock^0xffff;
    msg.motor_1_encoder_count	=   ((drive_packet_struct_t*)buffer)->encoder1^0xffff;
    msg.motor_2_encoder_count	=   ((drive_packet_struct_t*)buffer)->encoder2^0xffff;
    msg.motor_3_encoder_count	=   ((drive_packet_struct_t*)buffer)->encoder3^0xffff;
    msg.battery_volts			=  (((drive_packet_struct_t*)buffer)->battery_volts ^ 0xff)/5.0;
    msg.motor1_amps				= ((((drive_packet_struct_t*)buffer)->motor1_amps ^ 0xff)-128)/12.0;
    msg.motor2_amps				= ((((drive_packet_struct_t*)buffer)->motor2_amps ^ 0xff)-128)/12.0;
    msg.motor3_amps				= ((((drive_packet_struct_t*)buffer)->motor3_amps ^ 0xff)-128)/12.0;

    return msg;
}

void hw_interface_plugin_roboteq::roboteq_drive_packet::publishMsg(const ros::Publisher &rosPublisher, const messages::encoder_data &msg)
{
    rosPublisher.publish(msg);
}

void hw_interface_plugin_roboteq::roboteq_drive_packet::deserializeBufAndPublish(const ros::Publisher &rosPublisher, void * buffer)
{
    publishMsg(rosPublisher, deserializeBufIntoMsg(buffer));
}
