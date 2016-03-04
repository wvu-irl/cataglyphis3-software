/**
 * grabber_packet.h
 * Header file with definition of WVU-NSRR Packet Type #5
 */
#include <messages/GrabberFeedback.h>
#include "Buffer_Interface_class.h"
#include "Packet_Superclass.h"

class Grabber_Packet: public Buffer_Interface, public Packet
{
public:
	//Typedefs
	/**
	 * structure definition for WVU-NSRR packet 5
	 * (sent from grabber roboteq to i7)
	 */
	struct grabber_packet_t {
		char H1;
		char H2;
		uint8_t H3;
		uint8_t status_mux;
		uint16_t clock;
		uint8_t slider_pos_avg;
		uint8_t battery_volts;
		uint8_t motor1_amps;
		uint8_t motor2_amps;
		uint8_t motor3_amps;
		uint8_t drop_pos;
		uint8_t checksum;
	} __attribute__((packed));
	
	//Members
	/**
	 * union for common memory block for serial comm input buffer 
	 * and the packet structure variables
	 */
	union {
		char buf[100]; /* size buffer >3X data size */
		grabber_packet_t pkt;
	};
	messages::GrabberFeedback msg;
	
	//Methods
	Grabber_Packet(buffer_RW_t buffer_RW); //constructor to initialize Buffer_Interface
	void unpackMsg();
	void packMsg(const messages::GrabberFeedback::ConstPtr& msg);
	void subscribeMsg();
	void publishMsg(ros::Publisher* pub_ptr);
	void debug_print_packet_data(int res);  

};

