/**
 * nb2_3_to_i7_packet.h
 * Header file with definition of WVU-NSRR Packet Type #3 
 */
#include <messages/nb2_3_to_i7_msg.h>
#include "Buffer_Interface_class.h"
#include "Packet_Superclass.h"

class NB2_3_To_I7: public Buffer_Interface, public Packet
{
public:
	//Typedefs
	/**
	 * structure definition for WVU-NSRR packet 1 
	 * (sent from nb1 to i7)
	 */
	struct nb2_3_to_i7_t {
		char H1;
		char H2;
		uint8_t H3;
		uint16_t counter;
		uint32_t clock_reg_count;
		uint32_t clock_reg_reset_count;
		
		int32_t acc_x1;
		int32_t acc_y1;
		int32_t acc_z1;
		int32_t rate_p1;
		int32_t rate_q1;
		int32_t rate_r1;
		
		int32_t acc_x2;
		int32_t acc_y2;
		int32_t acc_z2;
		int32_t rate_p2;
		int32_t rate_q2;
		int32_t rate_r2;
		
		int32_t acc_x3;
		int32_t acc_y3;
		int32_t acc_z3;
		int32_t rate_p3;
		int32_t rate_q3;
		int32_t rate_r3;
		
        /*int32_t xda;
        int32_t yda;
        int32_t zda;
        int32_t xdv;
        int32_t ydv;
        int32_t zdv;*/
        
        //int16_t potValues[8];
        
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
		char buf[100]; /* size buffer >3X data size */
		nb2_3_to_i7_t pkt;
	};
	messages::nb2_3_to_i7_msg msg;
	
	//Methods
	NB2_3_To_I7(buffer_RW_t buffer_RW); //constructor to initialize Buffer_Interface
	void unpackMsg();
	void packMsg(const messages::nb2_3_to_i7_msg::ConstPtr& msg);
	void subscribeMsg();
	void publishMsg(ros::Publisher* pub_ptr);
	void debug_print_packet_data(int res);  

};

