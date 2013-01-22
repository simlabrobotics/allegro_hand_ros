#include <ros/ros.h>

#include "std_msgs/String.h"
//#include "std_msgs/Float32.h"
//#include "std_msgs/Int16.h"
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>


using namespace std;

#define DOF_JOINTS 16

// http://nehe.gamedev.net/article/msdn_virtualkey_codes/15009/
#define VK_SPACE 0x20
#define VK_LSHIFT 0xA0
 

#define KEYCODE_RIGHT 0x43 
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42

#define KEYCODE_0	0x30	
#define KEYCODE_1	0x31
#define KEYCODE_2	0x32
#define KEYCODE_3	0x33
#define KEYCODE_4	0x34
#define KEYCODE_5	0x35
#define KEYCODE_6	0x36
#define KEYCODE_7	0x37
#define KEYCODE_8	0x38
#define KEYCODE_9	0x39

#define KEYCODE_NUMPAD0	0x60
#define KEYCODE_NUMPAD1	0x61
#define KEYCODE_NUMPAD2	0x62
#define KEYCODE_NUMPAD3	0x63
#define KEYCODE_NUMPAD4	0x64
#define KEYCODE_NUMPAD5	0x65
#define KEYCODE_NUMPAD6	0x66
#define KEYCODE_NUMPAD7	0x67
#define KEYCODE_NUMPAD8	0x68
#define KEYCODE_NUMPAD9	0x69

#define KEYCODE_A	0x41
#define KEYCODE_B	0x42
#define KEYCODE_C	0x43
#define KEYCODE_D	0x44
#define KEYCODE_E	0x45
#define KEYCODE_F	0x46
#define KEYCODE_G	0x47
#define KEYCODE_H	0x48
#define KEYCODE_I	0x49
#define KEYCODE_J	0x4A
#define KEYCODE_K	0x4B
#define KEYCODE_L	0x4C
#define KEYCODE_M	0x4D
#define KEYCODE_N	0x4E
#define KEYCODE_O	0x4F
#define KEYCODE_P	0x50
#define KEYCODE_Q	0x51
#define KEYCODE_R	0x52
#define KEYCODE_S	0x53
#define KEYCODE_T	0x54
#define KEYCODE_U	0x55
#define KEYCODE_V	0x56
#define KEYCODE_W	0x57
#define KEYCODE_X	0x58
#define KEYCODE_Y	0x59
#define KEYCODE_Z	0x5A

#define KEYCODE_a	0x61
#define KEYCODE_b	0x62
#define KEYCODE_c	0x63
#define KEYCODE_d	0x64
#define KEYCODE_e	0x65
#define KEYCODE_f	0x66
#define KEYCODE_g	0x67
#define KEYCODE_h	0x68
#define KEYCODE_i	0x69
#define KEYCODE_j	0x6A
#define KEYCODE_k	0x6B
#define KEYCODE_l	0x6C
#define KEYCODE_m	0x6D
#define KEYCODE_n	0x6E
#define KEYCODE_o	0x6F
#define KEYCODE_p	0x70
#define KEYCODE_q	0x71
#define KEYCODE_r	0x72
#define KEYCODE_s	0x73
#define KEYCODE_t	0x74
#define KEYCODE_u	0x75
#define KEYCODE_v	0x76
#define KEYCODE_w	0x77
#define KEYCODE_x	0x78
#define KEYCODE_y	0x79
#define KEYCODE_z	0x7A

class AHKeyboard
{
public:
  AHKeyboard();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  int count_;//, angular_, l_scale_, a_scale_;
  float edit_;
  ros::Publisher cmd_pub_;
  
};

AHKeyboard::AHKeyboard():
count_(0),
edit_(0.0)
//  linear_(0),
// angular_(0),
//  l_scale_(2.0),
//  a_scale_(2.0)
{
//  nh_.param("scale_angular", a_scale_, a_scale_);
//  nh_.param("scale_linear", l_scale_, l_scale_);

  cmd_pub_ = nh_.advertise<std_msgs::String>("allegrohand/lib_cmd", 10);
}



//double curr_position[DOF_JOINTS];
//double desire_position[DOF_JOINTS];
// need to subscrive to the current position and modify
// then publish the desired position



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
  ros::init(argc, argv, "allegrohand_keyboard_cmd");
  AHKeyboard allegrohand_keyboard_cmd;

  signal(SIGINT,quit);

  allegrohand_keyboard_cmd.keyLoop();
  
  return(0);
}


void AHKeyboard::keyLoop()
{
  char c;
  bool dirty=false;
  bool jointUpdate=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");


  for(;;)
  {
  
    std_msgs::String msg;
    std::stringstream ss;
  
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    //linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
    printf("value: 0x%02X\n", c);

	switch(c)
    {
		case VK_SPACE:
        	ROS_DEBUG("space bar: PD Control");
        	//cmd_ = 4;
        	ss << "pdControl";
        	dirty = true;
        	break;
      	case KEYCODE_h:
        	ROS_DEBUG("h_key: Home");
        	//cmd_ = 1;
        	ss << "home";
        	dirty = true;
        	break;
      	case KEYCODE_r:
        	ROS_DEBUG("r_key: Ready");
        	//cmd_ = 2;
        	ss << "ready";
        	dirty = true;
        	break;
      	case KEYCODE_g:
        	ROS_DEBUG("g_key: Grasp (3 finger)");
        	//cmd_ = 3;
        	ss << "grasp_3";
        	dirty = true;
        	break;
      	case KEYCODE_p:
        	ROS_DEBUG("p_key: Pinch (index)");
        	//cmd_ = 4;
        	ss << "pinch_it";
        	dirty = true;
        	break;  
      	case KEYCODE_m:
        	ROS_DEBUG("m_key: Pinch (middle)");
        	//cmd_ = 4;
        	ss << "pinch_mt";
        	dirty = true;
        	break;  
		case KEYCODE_e:
        	ROS_DEBUG("e_key: Envelop");
        	//cmd_ = 4;
        	ss << "envelop";
        	dirty = true;
        	break;  
		case KEYCODE_o:
        	ROS_DEBUG("o_key: Servos Off");
        	//cmd_ = 4;
        	ss << "off";
        	dirty = true;
        	break; 
		case KEYCODE_s:
        	ROS_DEBUG("s_key: Save joint pos. for PD control");
        	//cmd_ = 4;
        	ss << "save";
        	dirty = true;
        	break;   
        case KEYCODE_z:
        	ROS_DEBUG("z_key: Edit the current set desired position incrementally");
        	//cmd_ = 4;
        	//ss << "edit";
        	
        	
        	
        	
        	
        	bool again = true;
        	
        	while(again){
        	
        	again = false;
        	
        	char finger;
        	finger_num = -1;
        	char knuckle;
        	knuckle_num = -1;
        	printf("\n\nPlease enter the # (1-4) of the finger to be edited.\n");
        	//printf("NOTE: You may not see your input in the console. Don't worry. Type then press ENTER.\n");
        	if(read(kfd, &finger, 1) < 0)
    		{
      			perror("read():");
      			exit(-1);
    		}
    		
        	printf("Please enter the # (1-4) of the joint to be edited.\n");
        	if(read(kfd, &knuckle, 1) < 0)
    		{
      			perror("read():");
      			exit(-1);
    		}
    		
    		if(finger == KEYCODE_1 || finger == KEYCODE_NUMPAD1) finger_num=1;
    		if(finger == KEYCODE_2 || finger == KEYCODE_NUMPAD2) finger_num=2;
    		if(finger == KEYCODE_3 || finger == KEYCODE_NUMPAD3) finger_num=3;
    		if(finger == KEYCODE_4 || finger == KEYCODE_NUMPAD4) finger_num=4;
    		
    		if(knuckle == KEYCODE_1 || knuckle == KEYCODE_NUMPAD1) knuckle_num=1;
    		if(knuckle == KEYCODE_2 || knuckle == KEYCODE_NUMPAD2) knuckle_num=2;
    		if(knuckle == KEYCODE_3 || knuckle == KEYCODE_NUMPAD3) knuckle_num=3;
    		if(knuckle == KEYCODE_4 || knuckle == KEYCODE_NUMPAD4) knuckle_num=4;
    		
    		printf("\nFinger: %d, Joint: %d.\nIs that correct? (c)ontinue, (r)etry or (q)uit...\n",finger_num, knuckle_num);
    		
    		char cont;
    		if(read(kfd, &cont, 1) < 0)
    		{
      			perror("read():");
      			exit(-1);
    		}
    		
    		if(cont == KEYCODE_q)
    		 printf("QUIT\n");// quit
    		
    		if(cont == KEYCODE_r)
    		again = true;
    		
    		if(cont == KEYCODE_c)
    		printf("CONTINUE\n");
    		
    		//else printf("That aint an option...");
    		
    		} // end while(again)
    		
    		finger_num+=-1;
    		knuckle_num+=-1;
    		int joint_num = finger_num*4+knuckle_num;
    		printf("\njoint: %d\n", joint_num);
    		
    		printf("Press the up and down arrow keys to adjust the current position of the chosen joint.");
    		
			// publish here
			// need to subscrive to the current position and modify
			// then publish the desired position
			
    		printf("broke out!");
    		getchar();
    		
    		        	
        	/*
        	int finger_;
   			cin >> finger_; 
   			cout << finger_ <<"\n";
   			
			printf("Please enter the # (1-4) of the joint to be edited, then press ENTER\n");
   			cin >> joint_; 
   			cout << joint_ <<"\n";   			   					
   			
   			finger_+=-1;
   			joint_+=-1;
        	int joint_number_ =  finger_+joint_;
        	cout << joint_number_;
        	*/
        	
        	
        	jointUpdate = true;
        	break;  	
        	
        	
/*
#define KEYCODE_1	0x31
#define KEYCODE_2	0x32
#define KEYCODE_3	0x33
#define KEYCODE_4	0x34
#define KEYCODE_5	0x35
#define KEYCODE_6	0x36
#define KEYCODE_7	0x37
#define KEYCODE_8	0x38
#define KEYCODE_9	0x39

#define KEYCODE_NUMPAD0	0x60
#define KEYCODE_NUMPAD1	0x61
#define KEYCODE_NUMPAD2	0x62
#define KEYCODE_NUMPAD3	0x63
#define KEYCODE_NUMPAD4	0x64
*/
        	      	        	
    }
   

    if(dirty ==true)
    {
      msg.data = ss.str();
      ROS_INFO("%s", msg.data.c_str());
      cmd_pub_.publish(msg);    
      dirty=false;
    }
    if(jointUpdate==true)
    {
    	// publish joint
    	jointUpdate=false;
    }
    
  }


  return;
}



