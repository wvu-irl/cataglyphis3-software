#ifndef BUFFER_INTERFACE_CLASS_H
#define BUFFER_INTERFACE_CLASS_H
#include<ros/ros.h>

enum buffer_RW_t {pktread, pktwrite};

class Buffer_Interface
{
public:
	// Members
	static bool reader;
	static char* read_buffer;
	static int rbuf_size;
	static char H1_def;
	static char H2_def;
	char H3_def = 's'; // Cannot be static
	char H4_def = 2;   // Cannot be static
	static bool writer;
	static char* write_buffer;
	static int wbuf_size;
	//Methods
	uint8_t computeChecksum(char* buf, int buf_len);
	uint8_t computeChecksumNot(char* buf, int buf_len);
};

bool Buffer_Interface::reader = false;
char* Buffer_Interface::read_buffer = 0; //set to null in order to detect unitialized buffers
int Buffer_Interface::rbuf_size = 0;
char Buffer_Interface::H1_def = 'A';
char Buffer_Interface::H2_def = 'z';
bool Buffer_Interface::writer = false;
char* Buffer_Interface::write_buffer = 0; //set to null in order to detect unitialized buffers
int Buffer_Interface::wbuf_size = 0;

uint8_t Buffer_Interface::computeChecksum(char* buf, int buf_len)
{
	uint8_t checksum_value;
	checksum_value = 0;
	for(int i=3; i<(buf_len-1); i++) // Use buf_len-1 because checksum byte itself should not be included in the checksum computation
	{
		checksum_value = (checksum_value + static_cast<uint8_t>(buf[i])) & 255;
		//ROS_ERROR("**!*!*!*!**!**!*** Intermediate checksum value = %d",checksum_value);
	}
	return checksum_value;
}

uint8_t Buffer_Interface::computeChecksumNot(char* buf, int buf_len)
{
	uint8_t checksum_value;
	checksum_value = 0;
	for(int i=2; i<(buf_len-1); i++) // Use buf_len-1 because checksum byte itself should not be included in the checksum computation
	{
		checksum_value = (checksum_value + static_cast<uint8_t>(buf[i]));
	}
	return ~checksum_value;
}

#endif /* BUFFER_INTERFACE_CLASS_H */
