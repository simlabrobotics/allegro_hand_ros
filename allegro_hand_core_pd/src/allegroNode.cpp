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
#include <math.h>
#include <iostream>
#include <string>

#include "controlAllegroHand.h"

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)


// Topics
#define JOINT_STATE_TOPIC "/allegroHand/joint_states"
#define JOINT_CMD_TOPIC "/allegroHand/joint_cmd"
#define EXT_CMD_TOPIC "/allegroHand/external_cmd"

double current_position[DOF_JOINTS] 			= {0.0};
double previous_position[DOF_JOINTS]			= {0.0};

double current_position_filtered[DOF_JOINTS] 	= {0.0};
double previous_position_filtered[DOF_JOINTS]	= {0.0};

double current_velocity[DOF_JOINTS] 			= {0.0};
double previous_velocity[DOF_JOINTS] 			= {0.0};
double current_velocity_filtered[DOF_JOINTS] 	= {0.0};

double desired_position[DOF_JOINTS]				= {0.0};
double desired_torque[DOF_JOINTS] 				= {0.0};

double k_p[DOF_JOINTS];
double k_d[DOF_JOINTS];



int frame = 0;

// Flags
int lEmergencyStop = 0;

boost::mutex *mutex;

// ROS Messages
ros::Publisher joint_state_pub;
ros::Subscriber joint_cmd_sub;		// handles external joint command (eg. sensor_msgs/JointState)
ros::Subscriber external_cmd_sub;	// handles any other type of eternal command (eg. std_msgs/String)
sensor_msgs::JointState msgJoint;
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






// In case of the Allegro Hand, this callback is processed
// every 0.003 seconds
void timerCallback(const ros::TimerEvent& event)
{
	// Calculate loop time;
	tnow = ros::Time::now();
	dt = 1e-9*(tnow - tstart).nsec;
	tstart = tnow;
	
/*  ================================= 
    =          UPDATE GAINS         = 
    ================================= */	
	// Read the gains.yaml file every iteration
	// This will allow us to update gains on the fly for tuning!
		
	// save last iteration info
	for(int i=0; i<DOF_JOINTS; i++) previous_position[i] = current_position[i];
	for(int i=0; i<DOF_JOINTS; i++) previous_position_filtered[i] = current_position_filtered[i];
	for(int i=0; i<DOF_JOINTS; i++) previous_velocity[i] = current_velocity[i];
		
		
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
    //double alpha = 0.6;
    //double beta = 0.198;	
	for(int i=0; i<DOF_JOINTS; i++)    
	{
    	current_position_filtered[i] = (0.6*current_position_filtered[i]) + (0.198*previous_position[i]) + (0.198*current_position[i]);
		current_velocity[i] = (current_position_filtered[i] - previous_position_filtered[i]) / dt;
		current_velocity_filtered[i] = (0.6*current_velocity_filtered[i]) + (0.198*previous_velocity[i]) + (0.198*current_velocity[i]);
	}
	
/*  ================================= 
    =        POSITION CONTROL       =   
    ================================= */
    if(frame>20) // give the low pass filters 20 iterations to build up good data.
    {	
		for(int i=0; i<DOF_JOINTS; i++)    
		{
			desired_torque[i] = k_p[i]*(desired_position[i]-current_position_filtered[i]) - k_d[i]*current_velocity_filtered[i];
			desired_torque[i] = desired_torque[i]/800.0;
		}
	}		


/*  ================================= 
    =   ADD VELOCITY SAT. CONTRL    =   
    ================================= */

		
	// PUBLISH current position, velocity and effort (torque)
	msgJoint.header.stamp 									= tnow;
	
	for(int i=0; i<DOF_JOINTS; i++) msgJoint.position[i] 	= current_position[i];
	
	//for(int i=0; i<DOF_JOINTS; i++) current_velocity[i] 	= (current_position[i] - previous_position[i])/dt;
	for(int i=0; i<DOF_JOINTS; i++) msgJoint.velocity[i] 	= current_velocity[i];
	
	for(int i=0; i<DOF_JOINTS; i++) msgJoint.effort[i] 		= desired_torque[i];
	
	joint_state_pub.publish(msgJoint);
		
		
	frame++;

} // end timerCallback








int main(int argc, char** argv)
{

k_p[0]  = 600;
k_p[1]  = 600;
k_p[2]  = 600;
k_p[3]  = 1000;
k_p[4]  = 600;
k_p[5]  = 600;
k_p[6]  = 600;
k_p[7]  = 1000;
k_p[8]  = 600;
k_p[9]  = 600;
k_p[10] = 600;
k_p[11] = 1000;
k_p[12] = 1000;
k_p[13] = 1000;
k_p[14] = 1000;
k_p[15] = 600;

k_d[0]  = 15;
k_d[1]  = 20;
k_d[2]  = 15;
k_d[3]  = 15;
k_d[4]  = 15;
k_d[5]  = 20;
k_d[6]  = 15;
k_d[7]  = 15;
k_d[8]  = 15;
k_d[9]  = 20;
k_d[10] = 15;
k_d[11] = 15;
k_d[12] = 30;
k_d[13] = 20;
k_d[14] = 20;
k_d[15] = 15;

desired_position[0]  = DEGREES_TO_RADIANS(0.0);
desired_position[1]  = DEGREES_TO_RADIANS(-10.0);
desired_position[2]  = DEGREES_TO_RADIANS(45.0);
desired_position[3]  = DEGREES_TO_RADIANS(45.0);
desired_position[4]  = DEGREES_TO_RADIANS(0.0);
desired_position[5]  = DEGREES_TO_RADIANS(-10.0);
desired_position[6]  = DEGREES_TO_RADIANS(45.0);
desired_position[7]  = DEGREES_TO_RADIANS(45.0);
desired_position[8]  = DEGREES_TO_RADIANS(5.0);
desired_position[9]  = DEGREES_TO_RADIANS(-5.0);
desired_position[10] = DEGREES_TO_RADIANS(50.0);
desired_position[11] = DEGREES_TO_RADIANS(45.0);
desired_position[12] = DEGREES_TO_RADIANS(60.0);
desired_position[13] = DEGREES_TO_RADIANS(25.0);
desired_position[14] = DEGREES_TO_RADIANS(15.0);
desired_position[15] = DEGREES_TO_RADIANS(45.0);





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
	msgJoint.name[0]  = "joint_0.0";
	msgJoint.name[1]  = "joint_1.0";
	msgJoint.name[2]  = "joint_2.0";
	msgJoint.name[3]  = "joint_3.0";
	msgJoint.name[4]  = "joint_4.0";
	msgJoint.name[5]  = "joint_5.0";
	msgJoint.name[6]  = "joint_6.0";
	msgJoint.name[7]  = "joint_7.0";
	msgJoint.name[8]  = "joint_8.0";
	msgJoint.name[9]  = "joint_9.0";
	msgJoint.name[10] = "joint_10.0";
	msgJoint.name[11] = "joint_11.0";
	msgJoint.name[12] = "joint_12.0";
	msgJoint.name[13] = "joint_13.0";
	msgJoint.name[14] = "joint_14.0";
	msgJoint.name[15] = "joint_15.0";

	
	// Get Allegro Hand information from parameter server
	// This information is found in the Hand-specific "zero.yaml" file from the allegro_hand_description package	
	string robot_name, whichHand, manufacturer, origin, serial, version;
	ros::param::get("/hand_info/robot_name",robot_name);
	ros::param::get("/hand_info/which_hand",whichHand);
	ros::param::get("/hand_info/manufacturer",manufacturer);
	ros::param::get("/hand_info/origin",origin);
	ros::param::get("/hand_info/serial",serial);
	ros::param::get("/hand_info/version",version);


	// Initialize CAN device
	canDevice = new controlAllegroHand();
	canDevice->init();
	usleep(3000);
	
	// Dump Allegro Hand information to the terminal	
	cout << endl << endl << robot_name << " " << version << endl << serial << " (" << whichHand << ")" << endl << manufacturer << endl << origin << endl << endl;

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


