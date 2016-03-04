/**
 * i7_to_nb1_packet.h
 * Header file with definition of WVU-NSRR Packet Type #2
 */
#include <messages/i7_to_nb1_msg.h>
#include "Buffer_Interface_class.h"
#include "Packet_Superclass.h"

class I7_To_NB1: public Buffer_Interface, public Packet
{
public:
	//Typedefs
	/**
	 * structure definition for WVU-NSRR packet 2
	 * (sent from i7 to nb1)
	 */
	struct i7_to_nb1_t {
		char H1;
		char H2;
		uint8_t H3;
		uint16_t counter;
		float x_position;
		float y_position;
		float heading;
		float north_angle;
		uint16_t waypoint_index;
		uint8_t waypoint_list_type;
		uint8_t samples_collected;
		uint8_t init_complete;
		uint8_t using_serial;
		uint8_t checksum;
	} __attribute__((packed));
	
	//Members
	/**
	 * union for common memory block for serial comm input buffer 
	 * and the packet structure variables
	 */
	union {
		char buf[100]; /* size buffer >3X data size */
		i7_to_nb1_t pkt;
	};
	messages::i7_to_nb1_msg msg;
	
	//Methods
	I7_To_NB1(buffer_RW_t buffer_RW); //constructor to initialize Buffer_Interface
	void unpackMsg();
	void packMsg(const messages::i7_to_nb1_msg::ConstPtr& msg);
	void subscribeMsg();
	void publishMsg(ros::Publisher* pub_ptr);
	void debug_print_packet_data(int res);  

};

