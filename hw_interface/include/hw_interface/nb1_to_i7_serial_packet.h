/**
 * nb1_to_i7_packet.h
 * Header file with definition of WVU-NSRR Packet Type #1 
 */
#include <messages/nb1_to_i7_serial_msg.h>
#include "Buffer_Interface_class.h"
#include "Packet_Superclass.h"

class NB1_To_I7_Serial: public Buffer_Interface, public Packet
{
public:
	//Typedefs
	/**
	 * structure definition for WVU-NSRR packet 1 
	 * (sent from nb1 to i7)
	 */
	struct nb1_to_i7_serial_t {
		char H1;
		char H2;
		uint8_t H3;
		uint16_t counter;
		uint32_t clock_reg_count;
		uint32_t clock_reg_reset_count;
		
		int32_t acc_x_mean;
		int32_t acc_y_mean;
		int32_t acc_z_mean;
		int32_t rate_p_mean;
		int32_t rate_q_mean;
		int32_t rate_r_mean;
        
        uint8_t imu_status; //bit shifted field containing info on IMU health
		uint8_t pause_switch;
		uint16_t main_loop_counter;
		uint8_t checksum;
	} __attribute__((packed));
	
	//Members
	/**
	 * union for common memory block for serial comm input buffer 
	 * and the packet structure variables
	 */
	union {
		char buf[250]; /* size buffer >3X data size */
		nb1_to_i7_serial_t pkt;
	};
	messages::nb1_to_i7_serial_msg msg;
	
	//Methods
	NB1_To_I7_Serial(buffer_RW_t buffer_RW); //constructor to initialize Buffer_Interface
	void unpackMsg();
	void packMsg(const messages::nb1_to_i7_serial_msg::ConstPtr& msg);
	void subscribeMsg();
	void publishMsg(ros::Publisher* pub_ptr);
	void debug_print_packet_data(int res);  

};

