#include <ros/ros.h>

#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>


using namespace std;

class AHKeyboard
{
public:
	AHKeyboard();
	void keyLoop();

private:

  
	ros::NodeHandle nh_;
	int count_;
	float edit_;
	ros::Publisher cmd_pub_;
  
};

	AHKeyboard::AHKeyboard():
	count_(0),
	edit_(0.0)
{
	cmd_pub_ = nh_.advertise<std_msgs::String>("/allegroHand/lib_cmd", 10);
}



int finger_num;
int knuckle_num;


int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "allegro_hand_keyboard_cmd");
  	AHKeyboard allegro_hand_keyboard_cmd;

  	signal(SIGINT,quit);

  	allegro_hand_keyboard_cmd.keyLoop();
  
  	return(0);
}


void AHKeyboard::keyLoop()
{
  	
	sleep(2);
	std::cout << std::endl;
	std::cout << "\t\t\t    Welcome to Allegro Hand!" 						<< std::endl;
	std::cout << " -----------------------------------------------------------------------" 	<< std::endl;
  	std::cout << "  You may transmit joint angles in realtime by publishing to the topic, "		<< std::endl;
  	std::cout << "  \t /allegroHand_<NUM>/joint_cmd."						<< std::endl;
  	std::cout << " -----------------------------------------------------------------------" 	<< std::endl;
 	std::cout << " -----------------------------------------------------------------------" 	<< std::endl;
	std::cout << "  Code originally by Dr. K.C. Chang & Alex Alspach."				<< std::endl;
	std::cout << "  Original code's repository: github.com/simlabrobotic/allegro_hand_ros"		<< std::endl;
	std::cout << "  Modified & trimmed to purpose by Abdul Rahman Dabbour."				<< std::endl;
	std::cout << "  This code's repository: github.com/thedabbour/allegro_hand_glove"		<< std::endl;
 	std::cout << " -----------------------------------------------------------------------" 	<< std::endl;
  return;
}
