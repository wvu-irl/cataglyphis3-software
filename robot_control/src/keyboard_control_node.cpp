#include <ros/ros.h>
#include <messages/ActuatorOut.h>
#include <messages/ExecInfo.h>
//#include <robot_control/Keys_Pressed.h>
//#include <messages/Servo_command.h>
//#include "robot_actions_class.h"
#define fully_open 1000
#define fully_closed -900
#define fully_dropped 1000
#define fully_raised -1000
#include <ncurses.h>

messages::ActuatorOut actuator_msg;
messages::ExecInfo exec_msg;
//robot_control::Keys_Pressed keys_msg;
//messages::Servo_command servo_msg;

int main(int argc, char **argv)
{
	ROS_INFO("Keyboard_Node - main() start");
	ros::init(argc, argv, "keyboard_node");
	ROS_INFO("Keyboard_Node - ros::init complete");
	ros::NodeHandle nh;
	ROS_INFO("Keyboard_Node - node handle created");
	
	ros::Rate loopRate(50); //set loop rate to 50Hz
	
	ros::Publisher actuator_pub = nh.advertise<messages::ActuatorOut>("control/actuatorout/all",1);
/*
	ros::Publisher keys_pub = nh.advertise<robot_control::Keys_Pressed>("control/keys_pressed",1);
    ros::Publisher servo_cam_pub = nh.advertise<messages::Servo_command>("camera_pan_servo_command",1);*/
    ros::Publisher exec_info_pub = nh.advertise<messages::ExecInfo>("control/exec/info",1);
	int ch; 
	#define initTimeouts 25
	int numTimeOuts = initTimeouts;
	
	int right[3] = {0}; //index 0 is the front wheel, 1 is middle, 2 is rear
	int left[3] = {0};
	int speed = 1000;	
	
	int grabberSlidePos = fully_open;
	int grabberDropPos  = fully_raised;
	
	float servoIncrement = 45.0;
	float servoAngleCmd = 180.0;
	
	bool done = false;
	bool killAll = false;

	initscr();			/* Start curses mode 		*/
	raw();				/* Line buffering disabled	*/
	keypad(stdscr, TRUE);		/* We get F1, F2 etc..		*/
	timeout(50);
	noecho();			/* Don't echo() while we do getch */
	resizeterm(24, 90);
    	while(!done && ros::ok())
    	{
	        ch = getch();			/* If raw() hadn't been called
					                 * we have to press enter before it
					                 * gets to the program 		*/
					 
	        switch(ch)
		    {	
		    
		    //drive control
		        case KEY_UP:
    		    //printw("The pressed key is up\n");
			        numTimeOuts = 0;
                    exec_msg.stopFlag = false;
                    exec_msg.turnFlag = false;
			        clear();printw("FORWARD!!");
    		        right[0] = speed; right[1] = speed; right[2] = speed;
    		        left[0] = speed; left[1] = speed; left[2] = speed;
				    break;
			    case KEY_RIGHT:
			    //printw("The pressed key is right\n");
			        numTimeOuts = 0;
                    exec_msg.stopFlag = false;
                    exec_msg.turnFlag = true;
			        clear();printw("RIGHT ROTATE");
                    right[0] = -speed/2; right[1] = -speed * 0.7347/2; right[2] = -speed/2;
    		        left[0] = speed/2; left[1] = speed * 0.7347/2; left[2] = speed/2;		    
				    break;
			    case KEY_LEFT:
			    //printw("The pressed key is left\n");
			        numTimeOuts = 0;
                    exec_msg.stopFlag = false;
                    exec_msg.turnFlag = true;
			        clear();printw("LEFT ROTATE");
                    right[0] = speed/2; right[1] = speed * 0.7347/2; right[2] = speed/2;
    		        left[0] = -speed/2; left[1] = -speed * 0.7347/2; left[2] = -speed/2;
				    break;
			    case KEY_DOWN:
			    //printw("The pressed key is down\n");
			        numTimeOuts = 0;
                    exec_msg.stopFlag = false;
                    exec_msg.turnFlag = false;
			        clear();printw("BACKWARD!!");
                    right[0] = -speed; right[1] = -speed; right[2] = -speed;
    		        left[0] = -speed; left[1] = -speed; left[2] = -speed;
				    break;
				    
			//grabber control
				case 119: //w grabber down
					numTimeOuts = 0;
                    exec_msg.stopFlag = true;
                    exec_msg.turnFlag = false;
					clear();printw("Grabber Down");
					grabberDropPos = fully_dropped; //gets these #define's from robot_actions
					break;
				case 115: //s grabber up
					numTimeOuts = 0;
                    exec_msg.stopFlag = true;
                    exec_msg.turnFlag = false;
					clear();printw("Grabber Up");
					grabberDropPos = fully_raised;
					break;
				case 101: //e grabber slides open
					numTimeOuts = 0;
                    exec_msg.stopFlag = true;
                    exec_msg.turnFlag = false;
					clear();printw("Grabber Slides Open");
					grabberSlidePos = fully_open;
					break;
				case 100: //d grabber slides closed
    				numTimeOuts = 0;
                    exec_msg.stopFlag = true;
                    exec_msg.turnFlag = false;
					clear();printw("Grabber Slides Closed");
					grabberSlidePos = fully_closed;
					break;	
			
			//camera control	
				case 111: //o (not zero) increase servo angle
    				numTimeOuts = 0;
                    exec_msg.stopFlag = true;
                    exec_msg.turnFlag = false;
				    clear();
				    servoAngleCmd += servoIncrement;
				    printw("Increase Servo Angle %f", servoAngleCmd);
				    if(servoAngleCmd > 358)
				    {
				        servoAngleCmd = 315.0;
				    }
				    break;
				case 112: //p decrease servo angle
    				numTimeOuts = 0;
                    exec_msg.stopFlag = true;
                    exec_msg.turnFlag = false;
				    clear();
				    servoAngleCmd += -servoIncrement;
				    printw("Decrease Servo Angle %f", servoAngleCmd);
				    if(servoAngleCmd < 0.5)
				    {
				        servoAngleCmd = 0.0;
				    }
				    break;
				case 32: //spacebar: take picture
				    numTimeOuts = 0;
                    exec_msg.stopFlag = true;
                    exec_msg.turnFlag = false;
				    clear();printw("Picture Time!");
				    //keys_msg.spacebar = true;
				    break;
				    
		    //stop flag
		        case 120: //x toggle stop flag
		            numTimeOuts = 0;
                    exec_msg.stopFlag = true;
                    exec_msg.turnFlag = false;
				    clear();
                    //exec_msg.stop_flag = exec_msg.stop_flag == 0 ? 1 : 0;
				    //printw("Stop Flag %s", exec_msg.stop_flag ? "ON" : "OFF");
				    break;
				    
            //capture common exit keys
				case 113: //if Q, close everything ROS
				    killAll = true;
				case 3: //if control-c, close just keyboard control
    				printw("Shutting Down");
    				right[0] = 0; right[1] = 0; right[2] = 0;
    		        left[0] = 0; left[1] = 0; left[2] = 0;
    		        //keys_msg.spacebar = false;
   				    done = true;
   				    break;
   				    
   			//timeout
				case -1: //timeout
				    numTimeOuts++;
                    exec_msg.stopFlag = true;
                    exec_msg.turnFlag = false;
				    if(numTimeOuts > 10)
				    {
				        clear();
				    }
				    if(numTimeOuts > initTimeouts)
				    {
				        printw("Keys 1, 2, 3 .. 0 are drive speed controls, 1 is the slowest : 0 is the fastest\n");
				        printw("Key 'p' rotates camera clockwise, Key 'o' rotates camera counter-clockwise\n");
                        //printw("Key 'x' toggles Stop Flag");
				        printw("Spacebar will take a picture and store it\n\n");
   				        printw("Arrow keys drive the robot\n");
				        printw("Key 'w' lowers the grabber, Key 's' raises the grabber\n");
				        printw("Key 'e' opens grabber slides, Key 'd' closes grabber slides\n\n");
				        printw("Key 'q' will close all rosnodes and end keyboard control\n");
				        printw("'Control + c' will close just keyboard control\n");
				    } 
    		        right[0] = 0; right[1] = 0; right[2] = 0;
    		        left[0] = 0; left[1] = 0; left[2] = 0;
    		        //keys_msg.spacebar = false;
				    break;
				    
		    //unknown key press
				default: //print
				    numTimeOuts = 0;
                    exec_msg.stopFlag = true;
                    exec_msg.turnFlag = false;
				    clear();printw("The pressed key is unknown %i", ch);
    		        right[0] = 0; right[1] = 0; right[2] = 0;
    		        left[0] = 0; left[1] = 0; left[2] = 0;
    	//	        keys_msg.spacebar = false;
    		        break;
		    }
            
            //drive speed control
            if(ch >= 49 && ch <= 57)
            {
                speed =  ((ch - 49)+1)/10.0 * 1000;
                clear();printw("Speed Change %i", speed);
            }
            else if(ch == 48)
            {
                speed = 1000;
            }
		    
			actuator_msg.fl_speed_cmd = left[0];
			actuator_msg.fr_speed_cmd = right[0];
			actuator_msg.ml_speed_cmd = left[1];
			actuator_msg.mr_speed_cmd = right[1];
			actuator_msg.bl_speed_cmd = left[2];
			actuator_msg.br_speed_cmd = right[2];
	        actuator_msg.slide_pos_cmd = grabberSlidePos;
	        actuator_msg.drop_pos_cmd = grabberDropPos;
	        
	        //servo_msg.camera_pan_angle_cmd = servoAngleCmd;
	        
	        actuator_pub.publish(actuator_msg);
            //keys_pub.publish(keys_msg);
            //servo_cam_pub.publish(servo_msg);
            exec_info_pub.publish(exec_msg);
	        
		    ros::spinOnce();
		    loopRate.sleep();
		
		}
					 
					
	refresh();			/* Print it on to the real screen */
    getch();			/* Wait for user input */
	endwin();			/* End curses mode		  */
	
	if(killAll)
	{
    //first, kill all nodes using 'rosnode kill -a' , this does not kill the master
    //second, sleep for 5 seconds
    //third, send SIGTERM signal to PID of roscore. ps -c roscore -o pid= returns the PID of roscore
	    system("rosnode kill -a && sleep 5 && kill -2 $( ps -C roslaunch -o pid= ) && sleep 2 && kill -2 $( ps -C roscore -o pid= )");
	}
	
    return 0;	
}
