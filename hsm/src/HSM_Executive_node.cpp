#include <ros/ros.h>
#include <hsm/HSM_Detection.h>
#include <hsm/HSM_Action.h>
#include <string>

enum executive_states_t {Monitoring, Recovering};
enum recovering_substates_t {Commanding,Confirming,Waiting};

struct detection_data_t
{
	std::string detector_type;
	std::string error_type;
	unsigned int consecutive_count;
};

detection_data_t detection_data;
bool detection_flag = false;
static executive_states_t executive_state = Monitoring;
static recovering_substates_t recovering_substate = Commanding;

void subCallback(const hsm::HSM_Detection::ConstPtr& msg_in);
void timerCallback(const ros::TimerEvent& event);

int main(int argc, char **argv)
{
	ros::init(argc,argv,"HSM_Executive");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<hsm::HSM_Action>("HSM_Act/TEST_DRIVER",1);
	ros::Subscriber sub = nh.subscribe<hsm::HSM_Detection>("HSM_Det/TEST_DRIVER",1,subCallback);
	hsm::HSM_Action msg_out;
	ros::Rate loop_rate(50);
	ros::Timer timer = nh.createTimer(ros::Duration(10),timerCallback,true);;
	
	while(ros::ok())
	{
		//std::cout << "executive_state = " << executive_state << std::endl;
		//std::cout << "error_type = " << detection_data.error_type << std::endl;
		//std::cout << "consecutive_count = " << detection_data.consecutive_count << std::endl;
		static std::string recovering_error_type;
		switch(executive_state)
		{
			case Monitoring:
				timer.stop();
				if(detection_flag)
				{
					if(detection_data.error_type == "Test Detection" && detection_data.consecutive_count>=50)
					{
						recovering_error_type = detection_data.error_type;
						executive_state = Recovering;
					}
					else executive_state = Monitoring;
				}
				else executive_state = Monitoring;
				detection_flag = false;
				break;
			case Recovering:
				if(recovering_error_type=="Test Detection")
				{
					switch(recovering_substate)
					{
						case Commanding:
							msg_out.command = "Test Command";
							pub.publish(msg_out);
							executive_state = Recovering;
							recovering_substate = Confirming;
							break;
						case Confirming: // For this test, does nothing except start timer
							timer.setPeriod(ros::Duration(10));
							timer.start();
							executive_state = Recovering;
							recovering_substate = Waiting;
							break;
						case Waiting:
							executive_state = Recovering;
							recovering_substate = Waiting;
							break;
					}
				}
				break;
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

void subCallback(const hsm::HSM_Detection::ConstPtr& msg_in)
{
	detection_data.detector_type = msg_in->detector_type;
	detection_data.error_type = msg_in->error_type;
//	detection_data.consecutive_count = msg_in->consecutive_count;
	detection_flag = true;
}

void timerCallback(const ros::TimerEvent& event)
{
	executive_state = Monitoring;
	recovering_substate = Commanding;
}
