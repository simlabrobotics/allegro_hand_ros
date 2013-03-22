/*
 * allegroNode.cpp
 *
 *  Created on: Nov 14, 2012
 *  Authors: Alex ALSPACH, Seungsu KIM
 */
 
// CONTROL LOOP TEMPLATE 
// Using  timer callback  

// For the most basic torque control algorithm, 
// you need only add code where it says:
	// =============================== //
	// = COMPUTE control torque here = //
	// =============================== //	
// in the timer callback below. 
// READ, COMPUTE, and WRITE and PUBLISH
// are contrained within this callback.
 
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
#include <iostream>
#include <string>

//#include "BHand/BHand.h"
#include "controlAllegroHand.h"

// Topics
#define JOINT_STATE_TOPIC "/allegroHand/joint_states"
#define JOINT_CMD_TOPIC "/allegroHand/joint_cmd"
#define EXT_CMD_TOPIC "/allegroHand/external_cmd"

#define JOINT_DESIRED_TOPIC "/allegroHand/joint_desired_states"
#define JOINT_CURRENT_TOPIC "/allegroHand/joint_current_states"


double desired_position[DOF_JOINTS]				= {0.0};
double current_position[DOF_JOINTS] 			= {0.0};
double previous_position[DOF_JOINTS]			= {0.0};
double current_position_filtered[DOF_JOINTS] 	= {0.0};
double previous_position_filtered[DOF_JOINTS]	= {0.0};

double desired_velocity[DOF_JOINTS]				= {0.0};
double current_velocity[DOF_JOINTS] 			= {0.0};
double previous_velocity[DOF_JOINTS] 			= {0.0};
double current_velocity_filtered[DOF_JOINTS] 	= {0.0};

double desired_torque[DOF_JOINTS] 				= {0.0};


std::string jointNames[DOF_JOINTS] 	= {    "joint_0.0",    "joint_1.0",    "joint_2.0",   "joint_3.0" , 
										   "joint_4.0",    "joint_5.0",    "joint_6.0",   "joint_7.0" , 
									  	   "joint_8.0",    "joint_9.0",    "joint_10.0",  "joint_11.0", 
										   "joint_12.0",   "joint_13.0",   "joint_14.0",  "joint_15.0" };

int frame = 0;

// Flags
int lEmergencyStop = 0;

boost::mutex *mutex;

// ROS Messages
ros::Publisher joint_state_pub;
ros::Publisher joint_desired_state_pub;
ros::Publisher joint_current_state_pub;

ros::Subscriber joint_cmd_sub;				// Handles external joint command (eg. sensor_msgs/JointState)
ros::Subscriber external_cmd_sub;			// Handles any other type of eternal command (eg. std_msgs/String)
sensor_msgs::JointState msgJoint_desired;	// Desired Position, Desired Velocity, Desired Torque
sensor_msgs::JointState msgJoint_current;	// Current Position, Current Velocity, NOTE: Current Torque Unavailable
sensor_msgs::JointState msgJoint;			// Collects most relavent information in one message: Current Position, Current Velocity, Control Torque
std::string  ext_cmd;

// ROS Time
ros::Time tstart;
ros::Time tnow;
double secs;
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
void extCmdCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("CTRL: Processing external command");
	ext_cmd = msg->data.c_str();

	// Compare the message received to an expected input
	if (ext_cmd.compare("comparisonString") == 0)
	{
		// Do something
	}    
	else
	{
		// Do something else
	}	
}


// In case of the Allegro Hand, this callback is processed every 0.003 seconds
void timerCallback(const ros::TimerEvent& event)
{
	// Calculate loop time;
	tnow = ros::Time::now();
	dt = 1e-9*(tnow - tstart).nsec;
	tstart = tnow;
		
	// save last joint position for velocity calc
	for(int i=0; i<DOF_JOINTS; i++) previous_position[i] = current_position[i];
		
		
		
		
/*  ================================= 
    =       CAN COMMUNICATION       = 
    ================================= */	
	canDevice->setTorque(desired_torque);		// WRITE joint torque
	lEmergencyStop = canDevice->update(); 		// Returns -1 in case of an error
	canDevice->getJointInfo(current_position);	// READ current joint positions

		
		
				
	// Stop program and shutdown node when Allegro Hand is switched off
	if( lEmergencyStop < 0 )
	{
		ROS_ERROR("\n\nAllegro Hand Node is Shutting Down! (Emergency Stop)\n");
		ros::shutdown();
	}


/*  ================================= 
	=       LOWPASS FILTERING       =   
	================================= */
	for(int i=0; i<DOF_JOINTS; i++)    
	{
		current_position_filtered[i] = (0.6*current_position_filtered[i]) + (0.198*previous_position[i]) + (0.198*current_position[i]);
		current_velocity[i] = (current_position_filtered[i] - previous_position_filtered[i]) / dt;
		current_velocity_filtered[i] = (0.6*current_velocity_filtered[i]) + (0.198*previous_velocity[i]) + (0.198*current_velocity[i]);
	}	





    if(frame>100) // give the low pass filters 100 iterations (0.03s) to build up good data.
	{

/*  ================================= 
	=  COMPUTE CONTROL TORQUE HERE  =   		// current_position_filtered --> desired_torque //
	================================= */	

	}





	// PUBLISH current position, velocity and effort (torque)
	msgJoint.header.stamp 		= tnow;	
	for(int i=0; i<DOF_JOINTS; i++)
	{
		msgJoint.position[i] 	= current_position_filtered[i];
		msgJoint.velocity[i] 	= current_velocity_filtered[i];
		msgJoint.effort[i] 		= desired_torque[i];
	}
	joint_state_pub.publish(msgJoint);
		
		
	frame++;

} // end timerCallback








int main(int argc, char** argv)
{
	using namespace std;
	
	ros::init(argc, argv, "allegro_hand_core");
	ros::Time::init();
	
	ros::NodeHandle nh;

	// Setup timer callback (ALLEGRO_CONTROL_TIME_INTERVAL = 0.003)
	ros::Timer timer = nh.createTimer(ros::Duration(0.003), timerCallback);

	mutex = new boost::mutex();

	// Publisher and Subscribers
	joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
	joint_cmd_sub = nh.subscribe(JOINT_CMD_TOPIC, 3, SetjointCallback);
	external_cmd_sub = nh.subscribe(EXT_CMD_TOPIC, 1, extCmdCallback);
	
	// Create arrays 16 long for each of the four joint state components
	msgJoint.position.resize(DOF_JOINTS);
	msgJoint.velocity.resize(DOF_JOINTS);
	msgJoint.effort.resize(DOF_JOINTS);
	msgJoint.name.resize(DOF_JOINTS);

	// Joint names (for use with joint_state_publisher GUI - matches URDF)
	for(int i=0; i<DOF_JOINTS; i++)	msgJoint.name[i] = jointNames[i];	

	
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
	

	// Initialize CAN device
	canDevice = new controlAllegroHand();
	canDevice->init();
	usleep(3000);
	
	// Dump Allegro Hand information to the terminal	
	cout << endl << endl << robot_name << " v" << version << endl << serial << " (" << whichHand << ")" << endl << manufacturer << endl << origin << endl << endl;

	// Initialize torque at zero
	for(int i=0; i<16; i++) desired_torque[i] = 0.0;

	// Start ROS time
	tstart = ros::Time::now();

	// Starts control loop, message pub/subs and all other callbacks
	ros::spin();			

	// Clean shutdown: shutdown node, shutdown can, be polite.
	nh.shutdown();
	delete canDevice;
	printf("\nAllegro Hand Node has been shut down. Bye!\n\n");
	return 0;
	
}


