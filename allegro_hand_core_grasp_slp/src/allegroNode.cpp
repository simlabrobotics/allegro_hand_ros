/*
 * allegroNode.cpp
 *
 *  Created on: Nov 14, 2012
 *  Authors: Alex ALSPACH, Seungsu KIM
 */
 
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

#include "BHand/BHand.h"
#include "controlAllegroHand.h"

// Topics
#define JOINT_STATE_TOPIC "/allegroHand/joint_states"
#define JOINT_CMD_TOPIC "/allegroHand/joint_cmd"
#define LIB_CMD_TOPIC "/allegroHand/lib_cmd"

double current_position[DOF_JOINTS];
double previous_position[DOF_JOINTS];
double current_velocity[DOF_JOINTS];
double desired_position[DOF_JOINTS];
double desired_torque[DOF_JOINTS];
std::string  lib_cmd;


int frame = 0;

// Flags
bool lIsBegin = false;
bool kill = false;	
bool pdControl = true;
int lEmergencyStop = 0;

boost::mutex *mutex;

// ROS Messages
ros::Publisher joint_state_pub;
ros::Subscriber joint_cmd_sub;
ros::Subscriber lib_cmd_sub;
sensor_msgs::JointState msgJoint;

// ROS Time
ros::Time tstart;
ros::Time tnow;
double secs;
double dt;

// Initialize BHand 
eMotionType gMotionType = eMotionType_NONE ;
// Uncomment one of the following according to which hand you are using
// Need to make this an option externally w/o recompiling
  BHand lBHand(eHandType_Left);
//BHand lBHand(eHandType_Right);


// Initialize CAN device		
controlAllegroHand *canDevice;



// Called when a desired joint position message is received
void SetjointCallback(const sensor_msgs::JointState& msg)
{
	// TODO check joint limits
	pdControl=true;

	mutex->lock();
	for(int i=0;i<DOF_JOINTS;i++) desired_position[i] = msg.position[i];
	mutex->unlock();

	lBHand.SetMotionType(eMotionType_JOINT_PD);
}

void libCmdCallback(const std_msgs::String::ConstPtr& msg)
{
ROS_INFO("I heard: [%s]", msg->data.c_str());
lib_cmd = msg->data.c_str();

// If PD Control is commanded, pdControl is set true
// so that the controll loop has access to desired position.
if (lib_cmd.compare("pdControl") == 0)
{
	lBHand.SetMotionType(eMotionType_JOINT_PD);
	pdControl = true;
}    
else
    pdControl = false;  

if (lib_cmd.compare("home") == 0) 
	lBHand.SetMotionType(eMotionType_HOME);
    
if (lib_cmd.compare("ready") == 0) 
	lBHand.SetMotionType(eMotionType_READY);

if (lib_cmd.compare("grasp_3") == 0) 
	lBHand.SetMotionType(eMotionType_GRASP_3);

if (lib_cmd.compare("grasp_4") == 0) 
	lBHand.SetMotionType(eMotionType_GRASP_4);
    
if (lib_cmd.compare("pinch_it") == 0) 
	lBHand.SetMotionType(eMotionType_PINCH_IT);

if (lib_cmd.compare("pinch_mt") == 0) 
	lBHand.SetMotionType(eMotionType_PINCH_MT); 	 

if (lib_cmd.compare("envelop") == 0) 
	lBHand.SetMotionType(eMotionType_ENVELOP); 

if (lib_cmd.compare("off") == 0) 
	lBHand.SetMotionType(eMotionType_NONE);

if (lib_cmd.compare("save") == 0) 
	for(int i=0; i<DOF_JOINTS; i++) desired_position[i] = current_position[i];

/*
eMotionType_NONE,				// motor power off
eMotionType_HOME,				// go to home position
eMotionType_READY,				// ready position for grasping
eMotionType_GRASP_3,			// grasping using 3 fingers
eMotionType_GRASP_4,			// grasping using 4 fingers
eMotionType_PINCH_IT,			// pinching using index finger and thumb
eMotionType_PINCH_MT,			// pinching using middle finger and thumb
eMotionType_ENVELOP,			// enveloping
eMotionType_JOINT_PD,			// joint pd control
eMotionType_OBJECT_MOVING,		//
eMotionType_PRE_SHAPE,			//
*/	

}



int main(int argc, char** argv)
{

	ros::init(argc, argv, "allegro_hand_core_grasp");
	ros::Time::init();

	ros::NodeHandle nh;

	// Setup sleep rate
	ros::Rate rate(1./ALLEGRO_CONTROL_TIME_INTERVAL);

	mutex = new boost::mutex();

	// Publisher and Subscribers
	joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
	joint_cmd_sub = nh.subscribe(JOINT_CMD_TOPIC, 3, SetjointCallback);
	lib_cmd_sub = nh.subscribe(LIB_CMD_TOPIC, 1000, libCmdCallback);

	// Create arrays 16 long for each of the four joint state components
	msgJoint.position.resize(DOF_JOINTS);
	msgJoint.velocity.resize(DOF_JOINTS);
	msgJoint.effort.resize(DOF_JOINTS);
	msgJoint.name.resize(DOF_JOINTS);

	// Joint names
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

	// Initialize BHand controller
	lBHand.SetTimeInterval(ALLEGRO_CONTROL_TIME_INTERVAL);

	// Initialize CAN device
	canDevice = new controlAllegroHand();
	canDevice->init();
	usleep(3000);

	// Initialize torque at zero
	for(int i=0; i<16; i++) desired_torque[i] = 0.0;

	// Set to false for first iteration of control loop (sets PD control at start pose)
	lIsBegin = false;

	// Start ROS time
	tstart = ros::Time::now();




	while(ros::ok())
	{
		// Calculate loop time;
		tnow = ros::Time::now();
		dt = 1e-9*(tnow - tstart).nsec;
		tstart = tnow;

		// save last joint position for velocity calc
		for(int i=0; i<DOF_JOINTS; i++) previous_position[i] = current_position[i];

		//// CAN Communication
		canDevice->setTorque(desired_torque);
		lEmergencyStop = canDevice->update();
                canDevice->getJointInfo(current_position);
		//// end CAN Communication

		if( lEmergencyStop < 0 )
		{
			// Stop program when Allegro Hand is switched off
			printf("\n\n\nEMERGENCY STOP.\n\n");
			ros::shutdown();
		}

		// compute control torque using Bhand library
		lBHand.SetJointPosition(current_position);

		// Run on FIRST iteration (sets PD control of intitial position)
		if( lIsBegin == false ){
			if(frame > 10){
				mutex->lock();
				for(int i=0; i<DOF_JOINTS; i++) desired_position[i] = current_position[i];
				mutex->unlock();

				lIsBegin = true;
				}

				// Start joint position control (Bhand)
				lBHand.SetMotionType(eMotionType_JOINT_PD);
				lBHand.UpdateControl((double)frame*ALLEGRO_CONTROL_TIME_INTERVAL);
				for(int i=0; i<DOF_JOINTS; i++) desired_torque[i] = 0.0;
			}
			else{			

				// BHAND Communication
				// desired joint positions are obtained from subscriber "joint_cmd_sub"
				// or maintatined as the initial positions from program start (PD control)
				// Also, other motions/grasps can be executed (envoked via "lib_cmd_sub")

				// Desired position only necessary if in PD Control mode	
				if (pdControl==true)
					lBHand.SetJointDesiredPosition(desired_position);

				// BHand lib control updated with time stamp	
				lBHand.UpdateControl((double)frame*ALLEGRO_CONTROL_TIME_INTERVAL);

				// Necessary torque obtained from Bhand lib
				lBHand.GetJointTorque(desired_torque);

				// Calculate joint velocity
				for(int i=0; i<DOF_JOINTS; i++)
					current_velocity[i] = (current_position[i] - previous_position[i])/dt;

				// current position, velocity and effort (torque) published
				msgJoint.header.stamp 									= tnow;
				for(int i=0; i<DOF_JOINTS; i++) msgJoint.position[i] 	= current_position[i];
				for(int i=0; i<DOF_JOINTS; i++) msgJoint.velocity[i] 	= current_velocity[i];
				for(int i=0; i<DOF_JOINTS; i++) msgJoint.effort[i] 		= desired_torque[i];
				joint_state_pub.publish(msgJoint);
			}

			frame++;	

			ros::spinOnce();
			rate.sleep();	
	}

	// Clean shutdown: shutdown node, shutdown can, be polite.
	nh.shutdown();
	delete canDevice;
	printf("\nBye.\n");
	return 0;
}

