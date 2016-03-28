#include <ros/ros.h>
#include <messages/NavFilterOut.h>

class Position
{
private:
	ros::Subscriber subscriber_nav;
	ros::NodeHandle node;
	
	//navigation callback for deadreckoning position
	void getNavCallback(const messages::NavFilterOut::ConstPtr &msg)
	{
		this->x = msg->x_position;
		this->y = msg->y_position;
		this->heading = msg->heading;
		counter++;
	}
public:
	double x,y,heading;
	int counter, prev_counter;
	Position()
	{
		x=0;
		y=0;
		heading=0;
		counter=0;
		prev_counter=0;
		subscriber_nav = node.subscribe("navigation/topic", 1, &Position::getNavCallback,this);
	};
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_node");
	ROS_INFO("test_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);

	Position position;

	while(ros::ok())
	{
		ROS_INFO("counter = %i", position.counter);
		loop_rate.sleep();
		ros::spinOnce();		
	}

	return 0;
}
