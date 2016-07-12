/**
 * drive_packet.h
 * Header file with definition of WVU-NSRR Packet Type #4
 */
#include <messages/encoder_data.h>
#include "Buffer_Interface_class.h"
#include "Packet_Superclass.h"

class Drive_Packet: public Buffer_Interface, public Packet
{
public:
	//Typedefs
	/**
	 * structure definition for WVU-NSRR packet 4
	 * (sent from drive roboteqs to i7)
	 */
	struct drive_packet_t {
		char H1;
		char H2;
		uint8_t H3;
		uint8_t counter;
		uint16_t clock;
		uint16_t encoder1;
		uint16_t encoder2;
		uint16_t encoder3;
		uint8_t battery_volts;
		uint8_t motor1_amps;
		uint8_t motor2_amps;
		uint8_t motor3_amps;
		uint8_t checksum;
	} __attribute__((packed));
	
	//Members
	/**
	 * union for common memory block for serial comm input buffer 
	 * and the packet structure variables
	 */
	union {
		char buf[100]; /* size buffer >3X data size */
		drive_packet_t pkt;
	};
	messages::encoder_data msg;
	
	//Methods
	Drive_Packet(buffer_RW_t buffer_RW); //constructor to initialize Buffer_Interface
	void unpackMsg();
	void packMsg(const messages::encoder_data::ConstPtr& msg);
	void subscribeMsg();
	void publishMsg(ros::Publisher* pub_ptr);
	void debug_print_packet_data(int res);  

};

