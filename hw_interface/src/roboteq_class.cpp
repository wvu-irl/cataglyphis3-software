#include "roboteq_class.h"

Roboteq_Class::Roboteq_Class()
{
	std::string node_type;
	std::string serial_port;
	if(ros::param::get("node_type", node_type)==false) node_type = "front_roboteq";
	if(ros::param::get("num_channels", num_channels)==false) num_channels = 2;
	// Initialize and open serial port
	reader = true;
	writer = true;
	port_ptr = new Serial_Port(_fixed_length_packet);
	motor_1_speed_cmd = 0;
	motor_2_speed_cmd = 0;
	motor_3_speed_cmd = 0;
	//read_buffer = read_buffer_local;
	write_buffer = write_buffer_local;
	//rbuf_size = 100;
	
	if(node_type=="front_roboteq")
	{
		pub = nh.advertise<messages::encoder_data>("roboteq/drivemotorin/front",1);
		sub = nh.subscribe<messages::ActuatorOut>("/control/actuatorout/all",1,&Roboteq_Class::frontCallback,this);
		packet_in_ptr = new Drive_Packet(pktread);
	}
	else if(node_type=="middle_roboteq")
	{
		pub = nh.advertise<messages::encoder_data>("roboteq/drivemotorin/middle",1);
		sub = nh.subscribe<messages::ActuatorOut>("/control/actuatorout/all",1,&Roboteq_Class::middleCallback,this);
		packet_in_ptr = new Drive_Packet(pktread);
	}
	else if(node_type=="back_roboteq")
	{
		pub = nh.advertise<messages::encoder_data>("roboteq/drivemotorin/back",1);
		sub = nh.subscribe<messages::ActuatorOut>("/control/actuatorout/all",1,&Roboteq_Class::backCallback,this);
		packet_in_ptr = new Drive_Packet(pktread);
	}
	else if(node_type=="left_roboteq")
	{
		pub = nh.advertise<messages::encoder_data>("roboteq/drivemotorin/left",1);
		sub = nh.subscribe<messages::ActuatorOut>("/control/actuatorout/all",1,&Roboteq_Class::leftCallback,this);
		packet_in_ptr = new Drive_Packet(pktread);
	}
	else if(node_type=="right_roboteq")
	{
		pub = nh.advertise<messages::encoder_data>("roboteq/drivemotorin/right",1);
		sub = nh.subscribe<messages::ActuatorOut>("/control/actuatorout/all",1,&Roboteq_Class::rightCallback,this);
		packet_in_ptr = new Drive_Packet(pktread);
	}
	else if(node_type=="grabber_roboteq")
	{
		pub = nh.advertise<messages::GrabberFeedback>("roboteq/grabberin/grabberin",1);
		sub = nh.subscribe<messages::ActuatorOut>("/control/actuatorout/all",1,&Roboteq_Class::grabberCallback,this);
		packet_in_ptr = new Grabber_Packet(pktread);
	}
	else ROS_ERROR("ROBOTEQ Invalid node type");
}

void Roboteq_Class::clearPluses()
{
	port_ptr->simple_read(2*num_channels);
}

void Roboteq_Class::frontCallback(const messages::ActuatorOut::ConstPtr& msg_in)
{
	motor_1_speed_cmd = msg_in->fl_speed_cmd;
	motor_2_speed_cmd = msg_in->fr_speed_cmd;
	callback_time = ros::Time::now().toSec();
}

void Roboteq_Class::middleCallback(const messages::ActuatorOut::ConstPtr& msg_in)
{
	motor_1_speed_cmd = msg_in->ml_speed_cmd;
	motor_2_speed_cmd = msg_in->mr_speed_cmd;
	callback_time = ros::Time::now().toSec();
}

void Roboteq_Class::backCallback(const messages::ActuatorOut::ConstPtr& msg_in)
{
	motor_1_speed_cmd = msg_in->bl_speed_cmd;
	motor_2_speed_cmd = msg_in->br_speed_cmd;
	callback_time = ros::Time::now().toSec();
}

void Roboteq_Class::leftCallback(const messages::ActuatorOut::ConstPtr& msg_in)
{
	motor_1_speed_cmd = msg_in->fl_speed_cmd;
	motor_2_speed_cmd = msg_in->ml_speed_cmd;
	motor_3_speed_cmd = msg_in->bl_speed_cmd;
	callback_time = ros::Time::now().toSec();
}

void Roboteq_Class::rightCallback(const messages::ActuatorOut::ConstPtr& msg_in)
{
	motor_1_speed_cmd = msg_in->fr_speed_cmd;
	motor_2_speed_cmd = msg_in->mr_speed_cmd;
	motor_3_speed_cmd = msg_in->br_speed_cmd;
	callback_time = ros::Time::now().toSec();
}

void Roboteq_Class::grabberCallback(const messages::ActuatorOut::ConstPtr& msg_in)
{
	slide_pos_cmd = msg_in->slide_pos_cmd;
	drop_pos_cmd = msg_in->drop_pos_cmd;
	grabber_stop_cmd = msg_in->grabber_stop_cmd;
	callback_time = ros::Time::now().toSec();
}

void Roboteq_Class::setMotorSpeeds()
{
	std::string motor_1_string;
	std::string motor_2_string;
	std::string motor_3_string;
	motor_1_string = "!G 1 " + boost::lexical_cast<std::string>(motor_1_speed_cmd) + "\r";
	motor_2_string = "!G 2 " + boost::lexical_cast<std::string>(motor_2_speed_cmd) + "\r";
	motor_3_string = "!G 3 " + boost::lexical_cast<std::string>(motor_3_speed_cmd) + "\r";
	write_buffer = const_cast<char*>(motor_1_string.c_str());
	wbuf_size = motor_1_string.size();
	port_ptr->port_write();
	ros::Duration(inter_command_delay).sleep();
	write_buffer = const_cast<char*>(motor_2_string.c_str());
	wbuf_size = motor_2_string.size();
	port_ptr->port_write();
	if(num_channels==3)
	{
		ros::Duration(inter_command_delay).sleep();
		write_buffer = const_cast<char*>(motor_3_string.c_str());
		wbuf_size = motor_3_string.size();
		port_ptr->port_write();
	}
	ros::Duration(inter_command_delay).sleep();
}

void Roboteq_Class::readEncoderValues()
{
	if(port_ptr->port_read()==true)
	{
		packet_in_ptr->unpackMsg();
		packet_in_ptr->publishMsg(&pub);
	}
}

void Roboteq_Class::setPrevValues()
{
	motor_1_encoder_count_prev = motor_1_encoder_count;
    motor_2_encoder_count_prev = motor_2_encoder_count;
}

void Roboteq_Class::setGrabberCommands()
{
	std::string slide_string;
	std::string drop_string;
	if(slide_pos_cmd!=slide_pos_cmd_prev)
	{
		slide_string = "!VAR 2 " + boost::lexical_cast<std::string>(slide_pos_cmd) + "\r";
		write_buffer = const_cast<char*>(slide_string.c_str());
		wbuf_size = slide_string.size();
		port_ptr->port_write();
		slide_pos_cmd_prev = slide_pos_cmd;
		ros::Duration(0.001).sleep();
		slide_string = "!VAR 1 0\r";
		write_buffer = const_cast<char*>(slide_string.c_str());
		wbuf_size = slide_string.size();
		port_ptr->port_write();
		slide_pos_cmd_prev = slide_pos_cmd;
	}
	ros::Duration(inter_command_delay).sleep();
	if(drop_pos_cmd!=drop_pos_cmd_prev)
	{
		drop_string = "!VAR 4 " + boost::lexical_cast<std::string>(drop_pos_cmd) + "\r";
		write_buffer = const_cast<char*>(drop_string.c_str());
		wbuf_size = drop_string.size();
		port_ptr->port_write();
		drop_pos_cmd_prev = drop_pos_cmd;
		ros::Duration(0.001).sleep();
		drop_string = "!VAR 3 0\r";
		write_buffer = const_cast<char*>(drop_string.c_str());
		wbuf_size = drop_string.size();
		port_ptr->port_write();
		drop_pos_cmd_prev = drop_pos_cmd;
	}
	ros::Duration(inter_command_delay).sleep();
}

void Roboteq_Class::grabberStop()
{
	std::string slide_string;
	std::string drop_string;
	slide_string = "!VAR 1 1\r";
	write_buffer = const_cast<char*>(slide_string.c_str());
	wbuf_size = slide_string.size();
	port_ptr->port_write();
	slide_pos_cmd_prev = 0;
	ros::Duration(inter_command_delay).sleep();
	drop_string = "!VAR 3 1\r";
	write_buffer = const_cast<char*>(drop_string.c_str());
	wbuf_size = drop_string.size();
	port_ptr->port_write();
	drop_pos_cmd_prev = 0;
}

void Roboteq_Class::readGrabberFeedback()
{
	if(port_ptr->port_read()==true)
	{
		packet_in_ptr->unpackMsg();
		packet_in_ptr->publishMsg(&pub);
	}
}

Roboteq_Class::~Roboteq_Class()
{
	
}
