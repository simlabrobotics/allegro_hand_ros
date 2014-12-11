/*
 * allegroNode.cpp
 *
 *  Created on: Feb 1, 2013
 *  Authors: Alex ALSPACH, Seungsu KIM
 */
 
// 20141210: kcchang: changed Duration to accomodate the hand's own CAN rate
// 20141211: kcchang: merged callback and polling

// JOINT SPACE POSITION CONTROL
// Using  timer callback 
 
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>

#include "ros/ros.h"
#include "ros/service.h"
#include "ros/service_server.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>

#include "controlAllegroHand.h"

// Topics
#define JOINT_STATE_TOPIC "/allegroHand/joint_states"
#define JOINT_CMD_TOPIC "/allegroHand/joint_cmd"
#define LIB_CMD_TOPIC "/allegroHand/lib_cmd"

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

double current_position[DOF_JOINTS] 			= {0.0};
double previous_position[DOF_JOINTS]			= {0.0};

double current_position_filtered[DOF_JOINTS] 	= {0.0};
double previous_position_filtered[DOF_JOINTS]	= {0.0};

double current_velocity[DOF_JOINTS] 			= {0.0};
double previous_velocity[DOF_JOINTS] 			= {0.0};
double current_velocity_filtered[DOF_JOINTS] 	= {0.0};

double desired_position[DOF_JOINTS]				= {0.0};
double desired_torque[DOF_JOINTS] 				= {0.0};

double k_p[DOF_JOINTS] =
{
	600.0,  600.0,  600.0, 1000.0,  // Default P Gains for PD Controller
	600.0,  600.0,  600.0, 1000.0,	// These gains are loaded if the 'gains_pd.yaml' file is not loaded
	600.0,  600.0,  600.0, 1000.0,
	1000.0, 1000.0, 1000.0,  600.0
};

double k_d[DOF_JOINTS] =
{
	15.0,   20.0,   15.0,   15.0,  // Default D Gains for PD Controller
	15.0,   20.0,   15.0,   15.0,	// These gains are loaded if the 'gains_pd.yaml' file is not loaded
	15.0,   20.0,   15.0,   15.0,
	30.0,   20.0,   20.0,   15.0
};
											
double home_pose[DOF_JOINTS] =
{
	0.0,  -10.0,   45.0,   45.0,  // Default (HOME) position (degrees)
	0.0,  -10.0,   45.0,   45.0,	// This position is loaded and set upon system start
	5.0,   -5.0,   50.0,   45.0,	// if no 'initial_position.yaml' parameter is loaded.
	60.0,   25.0,   15.0,   45.0
};

std::string pGainParams[DOF_JOINTS] =
{
	"~gains_pd/p/j00", "~gains_pd/p/j01", "~gains_pd/p/j02", "~gains_pd/p/j03", 
	"~gains_pd/p/j10", "~gains_pd/p/j11", "~gains_pd/p/j12", "~gains_pd/p/j13",
	"~gains_pd/p/j20", "~gains_pd/p/j21", "~gains_pd/p/j22", "~gains_pd/p/j23", 
	"~gains_pd/p/j30", "~gains_pd/p/j31", "~gains_pd/p/j32", "~gains_pd/p/j33"
};

std::string dGainParams[DOF_JOINTS] =
{
	"~gains_pd/d/j00", "~gains_pd/d/j01", "~gains_pd/d/j02", "~gains_pd/d/j03", 
	"~gains_pd/d/j10", "~gains_pd/d/j11", "~gains_pd/d/j12", "~gains_pd/d/j13",
	"~gains_pd/d/j20", "~gains_pd/d/j21", "~gains_pd/d/j22", "~gains_pd/d/j23", 
	"~gains_pd/d/j30", "~gains_pd/d/j31", "~gains_pd/d/j32", "~gains_pd/d/j33"
};
										
std::string initialPosition[DOF_JOINTS] =
{
	"~initial_position/j00", "~initial_position/j01", "~initial_position/j02", "~initial_position/j03", 
	"~initial_position/j10", "~initial_position/j11", "~initial_position/j12", "~initial_position/j13",
	"~initial_position/j20", "~initial_position/j21", "~initial_position/j22", "~initial_position/j23", 
	"~initial_position/j30", "~initial_position/j31", "~initial_position/j32", "~initial_position/j33"
};										

std::string jointNames[DOF_JOINTS] 	=
{
    "joint_0.0",    "joint_1.0",    "joint_2.0",   "joint_3.0" , 
	"joint_4.0",    "joint_5.0",    "joint_6.0",   "joint_7.0" , 
	"joint_8.0",    "joint_9.0",    "joint_10.0",  "joint_11.0", 
	"joint_12.0",   "joint_13.0",   "joint_14.0",  "joint_15.0"
};

long frame = 0;

// Flags
int lEmergencyStop = 0;
bool controlPD = false;

boost::mutex *mutex;

// ROS Messages
ros::Publisher joint_state_pub;
ros::Subscriber joint_cmd_sub;		// handles external joint command (eg. sensor_msgs/JointState)
ros::Subscriber lib_cmd_sub;	// handles any other type of eternal command (eg. std_msgs/String)
sensor_msgs::JointState msgJoint;
std::string  lib_cmd;

// ROS Time
ros::Time tstart;
ros::Time tnow;
double dt;

// Initialize CAN device		
controlAllegroHand *canDevice;

// Called when a desired joint position message is received
void SetjointCallback(const sensor_msgs::JointState& msg)
{
	mutex->lock();
	for(int i=0;i<DOF_JOINTS;i++) desired_position[i] = msg.position[i];
	mutex->unlock();	
}

// Called when an external (string) message is received
void libCmdCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());

	lib_cmd = msg->data.c_str();

	// Compare the message received to an expected input
	if (lib_cmd.compare("pdControl") == 0)
		controlPD = true;

	else if (lib_cmd.compare("home") == 0)
	{
		for(int i=0; i<DOF_JOINTS; i++)	desired_position[i] = DEGREES_TO_RADIANS(home_pose[i]);
		controlPD = true;
	}
	else if (lib_cmd.compare("off") == 0)
		controlPD = false;
	
	else if (lib_cmd.compare("save") == 0) 
		for (int i=0; i<DOF_JOINTS; i++) desired_position[i] = current_position[i];	
}

void computeDesiredTorque()
{
    /*  ================================= 
		=        POSITION CONTROL       =   
		================================= */
	if (controlPD)
	{
		for(int i=0; i<DOF_JOINTS; i++)    
		{
			desired_torque[i] = k_p[i]*(desired_position[i]-current_position_filtered[i]) - k_d[i]*current_velocity_filtered[i];
			desired_torque[i] = desired_torque[i]/canDevice->torqueConversion();
		}
	}
	else
	{
		for(int i=0; i<DOF_JOINTS; i++) desired_torque[i] = 0.0;
	}
}

void initController(const std::string& whichHand)
{
	// set gains_pd via gains_pd.yaml or to defaul values
	if (ros::param::has("~gains_pd"))
	{
		ROS_INFO("\n\nCTRL: PD gains loaded from param server.\n");
		for(int i=0; i<DOF_JOINTS; i++)
		{
			ros::param::get(pGainParams[i], k_p[i]);
			ros::param::get(dGainParams[i], k_d[i]);
			//printf("%f ", k_p[i]);
		}
		//printf("\n");
	}
	else
	{
		// gains will be loaded every control iteration
		ROS_WARN("\n\nCTRL: PD gains not loaded.\nCheck launch file is loading /parameters/gains_pd.yaml\nloading default PD gains...\n");
	}

	// set initial position via initial_position.yaml or to defaul values
	if (ros::param::has("~initial_position"))
	{
		ROS_INFO("\n\nCTRL: Initial Pose loaded from param server.\n");
		for(int i=0; i<DOF_JOINTS; i++)
		{
			ros::param::get(initialPosition[i], desired_position[i]);
			//ROS_INFO("%s, %f\n",initialPosition[i].c_str(), desired_position[i]);
			desired_position[i] = DEGREES_TO_RADIANS(desired_position[i]);
		}
	}
	else
	{
		ROS_WARN("\n\nCTRL: Initial postion not loaded.\nCheck launch file is loading /parameters/initial_position.yaml\nloading Home position instead...\n");
		// Home position
		for(int i=0; i<DOF_JOINTS; i++)	desired_position[i] = DEGREES_TO_RADIANS(home_pose[i]);										
	}
	controlPD = false;

	printf("*************************************\n");
	printf("      Joint PD Control Method        \n");
	printf("-------------------------------------\n");
    printf("  Only 'H', 'O', 'S', 'Space' works. \n");
	printf("*************************************\n");	
}

void cleanController()
{
}

void publishData()
{
	// current position, velocity and effort (torque) published
	msgJoint.header.stamp = tnow;
	for (int i=0; i<DOF_JOINTS; i++)
	{
		msgJoint.position[i] 			= current_position_filtered[i];
		msgJoint.velocity[i] 			= current_velocity_filtered[i];
		msgJoint.effort[i] 				= desired_torque[i];
	}
	joint_state_pub.publish(msgJoint);
}

void updateWriteReadCAN()
{
	/* ================================== 
	   =        CAN COMMUNICATION         =   
	   ================================== */
	canDevice->setTorque(desired_torque);
	lEmergencyStop = canDevice->Update();
	canDevice->getJointInfo(current_position);		

	if (lEmergencyStop < 0)
	{
		// Stop program when Allegro Hand is switched off
		//printf("\n\n\nEMERGENCY STOP.\n\n");
		ROS_ERROR("\n\nAllegro Hand Node is Shutting Down! (Emergency Stop)\n");
		ros::shutdown();
	}
}

void updateController()
{
	// Calculate loop time;
	tnow = ros::Time::now();
	dt = 1e-9*(tnow - tstart).nsec;
	tstart = tnow;
		
	// save last iteration info
	for (int i=0; i<DOF_JOINTS; i++)
	{
		previous_position[i] = current_position[i];
		previous_position_filtered[i] = current_position_filtered[i];
		previous_velocity[i] = current_velocity[i];
	}

	updateWriteReadCAN();
	
	/* ================================== 
	   =         LOWPASS FILTERING        =   
	   ================================== */
	for (int i=0; i<DOF_JOINTS; i++)    
	{
		current_position_filtered[i] = (0.6*current_position_filtered[i]) + (0.198*previous_position[i]) + (0.198*current_position[i]);
		current_velocity[i] = (current_position_filtered[i] - previous_position_filtered[i]) / dt;
		current_velocity_filtered[i] = (0.6*current_velocity_filtered[i]) + (0.198*previous_velocity[i]) + (0.198*current_velocity[i]);
		current_velocity[i] = (current_position[i] - previous_position[i]) / dt;
	}
	
	computeDesiredTorque();	

	publishData();

	frame++;
} 

// In case of the Allegro Hand, this callback is processed every 0.003 seconds
void timerCallback(const ros::TimerEvent& event)
{
	updateController();
}

int main(int argc, char** argv)
{	
	using namespace std;
	
	ros::init(argc, argv, "allegro_hand_core_grasp");
	ros::Time::init();
	
	ros::NodeHandle nh;

	mutex = new boost::mutex();

	// Publisher and Subscribers
	joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
	joint_cmd_sub = nh.subscribe(JOINT_CMD_TOPIC, 3, SetjointCallback);
	lib_cmd_sub = nh.subscribe(LIB_CMD_TOPIC, 1, libCmdCallback);
	
	// Create arrays 16 long for each of the four joint state components
	msgJoint.position.resize(DOF_JOINTS);
	msgJoint.velocity.resize(DOF_JOINTS);
	msgJoint.effort.resize(DOF_JOINTS);
	msgJoint.name.resize(DOF_JOINTS);

	// Joint names (for use with joint_state_publisher GUI - matches URDF)
	for (int i=0; i<DOF_JOINTS; i++)	msgJoint.name[i] = jointNames[i];	
	
	// Get Allegro Hand information from parameter server
	// This information is found in the Hand-specific "zero.yaml" file from the allegro_hand_description package	
	string robot_name, whichHand, manufacturer, origin, serial;
	double version;
	ros::param::get("~hand_info/robot_name",robot_name);
	ros::param::get("~hand_info/which_hand",whichHand);
	ros::param::get("~hand_info/manufacturer",manufacturer);
	ros::param::get("~hand_info/origin",origin);
	ros::param::get("~hand_info/serial",serial);
	ros::param::get("~hand_info/version",version);

	// Dump Allegro Hand information to the terminal	
	cout << endl << endl << robot_name << " v" << version << endl << serial << " (" << whichHand << ")" << endl << manufacturer << endl << origin << endl << endl;
		
	// Initialize CAN device
	canDevice = new controlAllegroHand();
	canDevice->init();
	usleep(3000);
		
	// Initialize torque at zero
	for (int i=0; i<DOF_JOINTS; i++) desired_torque[i] = current_velocity[i] = 0.0;

	// Initialize current pos to Hand pos
	updateWriteReadCAN();

	initController(whichHand);
	
	// Start ROS time
	tstart = ros::Time::now();
	
	// Starts control loop, message pub/subs and all other callbacks
	printf("\n\nStart controller with polling:=");
	
	if (argv[1] == std::string("true")) //polling:=true
	{
		printf("true\n");
		while (ros::ok())
		{
			updateController();
			ros::spinOnce();
		}
	}
	else
	{
		printf("false\n");
		// Setup timer callback
		ros::Timer timer = nh.createTimer(ros::Duration(0.001), timerCallback); //KCX
		ros::spin();		
	}

	// Prompt user to press enter to quit node for viewing failed launch or system after CAN error
	//cout << "Press Enter to Continue";
	//cin.ignore();

	// Clean shutdown: shutdown node, shutdown can, be polite.
	nh.shutdown();
	cleanController();
	delete canDevice;
	//printf("\nBye.\n");
	printf("\nAllegro Hand Node has been shut down. Bye!\n\n");
	return 0;
}
