 #ifndef ROBOTEQ_DRIVE_PACKET_HPP__
#define ROBOTEQ_DRIVE_PACKET_HPP__

#include <ros/ros.h>
#include <messages/encoder_data.h>

namespace hw_interface_plugin_roboteq {

    namespace roboteq_drive_packet {
    
        const char * HEADER = "Az";
        const char * FOOTER = "P1fC";

        struct drive_packet_struct_t 
        {
            uint8_t header[2];
            uint8_t senderID;
            uint8_t counter;
            uint16_t clock;
            uint16_t encoder1;
            uint16_t encoder2;
            uint16_t encoder3;
            uint8_t battery_volts;
            uint8_t motor1_amps;
            uint8_t motor2_amps;
            uint8_t motor3_amps;
            uint8_t iMarkers4_10;
            uint8_t iMarkers11_16;
            uint32_t crc32;
            uint8_t iMarkerCRC;
            uint8_t footer[4];
        } __attribute__((packed));

        const int PACKET_SIZE_MINUS_CRC_AND_FOOTER = sizeof(drive_packet_struct_t) - 8;
        
        messages::encoder_data deserializeBufIntoMsg(void * buffer);
        
        void publishMsg(const ros::Publisher &rosPublisher, const messages::encoder_data &msg);
        
        void deserializeBufAndPublish(const ros::Publisher &rosPublisher, void *buffer);
    }
}

#endif //ROBOTEQ_DRIVE_PACKET_HPP__
