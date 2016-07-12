#ifndef SERIAL_PORT_CLASS_H
#define SERIAL_PORT_CLASS_H
/**
 * Serial_Port_class.h
 */
//#include <sys/types.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <sys/time.h>
#include "Buffer_Interface_class.h"
#include "Port_Superclass.h"
//#include <hsm/HSM_Detector.h>

/**
 * Serial port configuration constants
 */
//#define BAUDRATE B115200
#define _POSIX_SOURCE 1           // POSIX compliant source
enum port_mode_t {_fixed_length_packet, _roboteq};
enum checksum_type_t {_and, _not};

class Serial_Port: public Buffer_Interface, public Port
{
public:
	//   Members 
	/**
	 * fd - file descriptor, reference to opened port
	 * res - return value, number of characters read
	 * oldtio - prior port setttings
	 * newtio - new port settings
	 */
	std::string MODEMDEVICE;
	int BAUDRATE;
	speed_t baud_rate;
	int fd, res;
	struct termios oldtio, newtio;
	enum read_mode_t {single_char,full_packet} read_mode = single_char;
	enum single_char_step_t {init_s,find_H1,find_H2,find_H3,read_body} single_char_step = init_s;
	enum full_packet_step_t {init_f,run} full_packet_step = init_f;
	int index = 3;
	
	char one_buff[1];
//	HSM_Detector det;
	checksum_type_t checksum_type;
	uint8_t checksum_value = 0;
	//Methods
	Serial_Port(port_mode_t port_mode_input); //constructor - opens port
	Serial_Port();
	~Serial_Port();       					  //destructor - closes port
	bool port_read();
	bool port_write();
	bool simple_read(int read_length);
	int  flushBuffer();
private:
	//Members
	port_mode_t port_mode;
	struct timeval start_time, current_time;
	long timeout_usec = 500000;
	useconds_t read_delay_usec = 50000;

	//Methods
	bool read_timeout();
};

#endif /* SERIAL_PORT_CLASS_H */
