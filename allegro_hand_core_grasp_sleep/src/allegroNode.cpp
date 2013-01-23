/*
 * allegroNode.cpp
 *
 *  Created on: Nov 14, 2012
 *  Authors: Alex ALSPACH, Seungsu KIM
 */
 
<<<<<<< HEAD
 // Add something like, if the time is bigger than nominal (0.003)+tolerance, sprit it out to the terminal. Also, consider dropping the file writing all together and saving bag file ros style.
 //Make a launch file that will launch the bag file saver along with allegro. What else is the launch file good for? time to start integrating the kinect???
 
 
 
 // NEED TO BE ABLE TO ACCESS THE STRING OR INTEGER FROM THE MESSAGE
 // CAN PRINT IT BUT CANT USE IT
 
 

/* 
figure out how to use this to pull out joint
limits and use them to saturate joint commands.
*/ 
//#include "urdf_interface/joint.h"
 
=======
>>>>>>> f52340901b8ee1d7e1cd864848f79032fccea222
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

<<<<<<< HEAD
#define JOINT_STATE_TOPIC "/allegro/joint_states"
#define JOINT_CMD_TOPIC "/allegro/joint_cmd"
#define LIB_CMD_TOPIC "/allegrohand/lib_cmd"
//#define JOINT_CMD_TOPIC "joint_states"

double curr_position[DOF_JOINTS];
double curr_position_pre[DOF_JOINTS];
double curr_velocity[DOF_JOINTS];
double curr_torque[DOF_JOINTS];
double desire_position[DOF_JOINTS];
double desire_torque[DOF_JOINTS];
double out_torque[DOF_JOINTS];
std::string  lib_cmd;

double secs;

int frame = 0;
int lEmergencyStop = 0;

double dt;

bool lIsBegin = false;
bool kill = false;	
bool pdControl = true;

FILE *fid;
FILE *fidTime;
=======
// Topics
#define JOINT_STATE_TOPIC "/allegroHand/joint_states"
#define JOINT_CMD_TOPIC "/allegroHand/joint_cmd"
#define LIB_CMD_TOPIC "/allegroHand/lib_cmd"

double current_position[DOF_JOINTS];
double previous_position[DOF_JOINTS];
double current_velocity[DOF_JOINTS];
double current_torque[DOF_JOINTS];
double desired_position[DOF_JOINTS];
double desired_torque[DOF_JOINTS];
std::string  lib_cmd;


int frame = 0;

// Flags
bool lIsBegin = false;
bool kill = false;	
bool pdControl = true;
int lEmergencyStop = 0;
>>>>>>> f52340901b8ee1d7e1cd864848f79032fccea222

boost::mutex *mutex;

// ROS Messages
ros::Publisher joint_state_pub;
ros::Subscriber joint_cmd_sub;
ros::Subscriber lib_cmd_sub;
sensor_msgs::JointState msgJoint;

// ROS Time
ros::Time tstart;
ros::Time tnow;
<<<<<<< HEAD
=======
double secs;
double dt;
>>>>>>> f52340901b8ee1d7e1cd864848f79032fccea222

// Initialize BHand 
eMotionType gMotionType = eMotionType_NONE ;
// Uncomment one of the following according to which hand you are using
<<<<<<< HEAD
//BHand lBHand(eHandType_Right);
  BHand lBHand(eHandType_Left);
=======
// Need to make this an option externally w/o recompiling
  BHand lBHand(eHandType_Left);
//BHand lBHand(eHandType_Right);

>>>>>>> f52340901b8ee1d7e1cd864848f79032fccea222

// Initialize CAN device		
controlAllegroHand *canDevice;



// Called when a desired joint position message is received
void SetjointCallback(const sensor_msgs::JointState& msg)
{
	// TODO check joint limits
	pdControl=true;
<<<<<<< HEAD
    //lBHand.SetMotionType(eMotionType_JOINT_PD);
	mutex->lock();
	for(int i=0;i<DOF_JOINTS;i++) desire_position[i] = msg.position[i];
	mutex->unlock();
	//printf("Set to PD Control mode to execute Joint Pos. command");
	//printf("%f",desire_position[0]);
=======
	
	mutex->lock();
	for(int i=0;i<DOF_JOINTS;i++) desired_position[i] = msg.position[i];
	mutex->unlock();
	
>>>>>>> f52340901b8ee1d7e1cd864848f79032fccea222
	lBHand.SetMotionType(eMotionType_JOINT_PD);
}

void libCmdCallback(const std_msgs::String::ConstPtr& msg)
{
ROS_INFO("I heard: [%s]", msg->data.c_str());
lib_cmd = msg->data.c_str();

<<<<<<< HEAD
// if PD Control is commanded, pdControl is set true
=======
// If PD Control is commanded, pdControl is set true
>>>>>>> f52340901b8ee1d7e1cd864848f79032fccea222
// so that the controll loop has access to desired position.
if (lib_cmd.compare("pdControl") == 0)
{
	lBHand.SetMotionType(eMotionType_JOINT_PD);
	pdControl = true;
<<<<<<< HEAD
    printf("PD\n");
=======
>>>>>>> f52340901b8ee1d7e1cd864848f79032fccea222
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
<<<<<<< HEAD
	for(int i=0; i<DOF_JOINTS; i++) desire_position[i] = curr_position[i];
	
/*
eMotionType_NONE,				///< power off
eMotionType_HOME,				///< go to home position
eMotionType_READY,				///< finger position move motion (ready)
eMotionType_PRE_SHAPE,			///<
eMotionType_GRASP_3,			///< grasping using 3 fingers
eMotionType_GRASP_4,			///< grasping using 4 fingers
eMotionType_PINCH_IT,			///< pinching using index finger and thumb
eMotionType_PINCH_MT,			///< pinching using middle finger and thumb
eMotionType_OBJECT_MOVING,		///<
eMotionType_ENVELOP,			///< enveloping
eMotionType_JOINT_PD,			///< joint pd control
=======
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
>>>>>>> f52340901b8ee1d7e1cd864848f79032fccea222
*/	
	
}


<<<<<<< HEAD
void timerCallback(const ros::TimerEvent& event)
{
	// Calculate loop time;
	tnow = ros::Time::now();
	dt = 1e-9*(tnow - tstart).nsec; //(tnow - tstart).nsec; 
	tstart = tnow;
		
	// save last joint position for velocity calc
	for(int i=0; i<DOF_JOINTS; i++) curr_position_pre[i] = curr_position[i];
		
	//// CAN Communication
	canDevice->setTorque(desire_torque);
	lEmergencyStop = canDevice->update();
	canDevice->getJointInfo(curr_position, curr_torque);
	//// end CAN Communication
				
	if( lEmergencyStop < 0 )
	{
		// Stop program when Allegro Hand is switched off
		printf("\n\n\nEMERGENCY STOP.\n\n");
		ros::shutdown();
	}
	
	/*// print to file (doesnt work)
	for(int i=0; i<DOF_JOINTS; i++ ) fprintf(fid, "%lf ", desire_position[i]);
	for(int i=0; i<DOF_JOINTS; i++ ) fprintf(fid, "%lf ", curr_position[i]);
	for(int i=0; i<DOF_JOINTS; i++ ) fprintf(fid, "%lf ", desire_torque[i]);
	fprintf(fid, "%lf\n",dt);
	*/	

	// compute torque using Bhand library
	lBHand.SetJointPosition(curr_position);
	
	// Run on FIRST iteration (sets PD control and intitial set position)
	if( lIsBegin == false ){
		if(frame > 10){
			mutex->lock();
			for(int i=0; i<DOF_JOINTS; i++) desire_position[i] = curr_position[i];
			mutex->unlock();

			lIsBegin = true;
			}
			
			// Start joint position control (Bhand)
			lBHand.SetMotionType(eMotionType_JOINT_PD);
			lBHand.UpdateControl((double)frame*ALLEGRO_CONTROL_TIME_INTERVAL);
			for(int i=0; i<DOF_JOINTS; i++) desire_torque[i] = 0.0;
		}
		else{
			// BHAND Communication
			// desired position is obtained from subscriber "joint_cmd_sub"
			// or maintatined as the initial position from program start
			// Also, other motions/grasps are performed and comtrolled
			// obtained via "lib_cmd_sub"
			
			// desired position only necessary if in PD Control mode
			if (pdControl==true)
				lBHand.SetJointDesiredPosition(desire_position);
			
			lBHand.UpdateControl((double)frame*ALLEGRO_CONTROL_TIME_INTERVAL);
			
			// Necessary torque obtained from Bhand lib
			lBHand.GetJointTorque(out_torque);
			for(int i=0; i<DOF_JOINTS; i++) desire_torque[i] = out_torque[i];
			// end BHAND Communication
			
			// Calculate joint velocity
			for(int i=0; i<DOF_JOINTS; i++) curr_velocity[i] = (curr_position[i] - curr_position_pre[i])/dt;
			
			// current position, velocity and effort (torque) published
			msgJoint.header.stamp = tnow;
			for(int i=0; i<DOF_JOINTS; i++) msgJoint.position[i] = curr_position[i];
			for(int i=0; i<DOF_JOINTS; i++) msgJoint.velocity[i] = curr_velocity[i];
			for(int i=0; i<DOF_JOINTS; i++) msgJoint.effort[i] = desire_torque[i];
			joint_state_pub.publish(msgJoint);
		}
		
		frame++;
} // end timerCallback








int main(int argc, char** argv)
{
	// Open up text files for writing data (currently cant write)
	fid = fopen("./allegro_log.txt", "w+");
	fidTime = fopen("./allegro_Timelog.txt", "w+");

	ros::init(argc, argv, "allegro");
=======

int main(int argc, char** argv)
{

	ros::init(argc, argv, "allegro_hand_core_grasp");
>>>>>>> f52340901b8ee1d7e1cd864848f79032fccea222
	ros::Time::init();
	
	ros::NodeHandle nh;

<<<<<<< HEAD
	// Setup timer callback
	ros::Timer timer = nh.createTimer(ros::Duration(0.003), timerCallback);

	mutex = new boost::mutex();

=======
	// Setup sleep rate
	ros::Rate rate(1./ALLEGRO_CONTROL_TIME_INTERVAL);;

	mutex = new boost::mutex();

	// Publisher and Subscribers
>>>>>>> f52340901b8ee1d7e1cd864848f79032fccea222
	joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
	joint_cmd_sub = nh.subscribe(JOINT_CMD_TOPIC, 3, SetjointCallback);
	lib_cmd_sub = nh.subscribe(LIB_CMD_TOPIC, 1000, libCmdCallback);
	
	// Create arrays 16 long for each of the four joint state components
	msgJoint.position.resize(DOF_JOINTS);
	msgJoint.velocity.resize(DOF_JOINTS);
	msgJoint.effort.resize(DOF_JOINTS);
	msgJoint.name.resize(DOF_JOINTS);
<<<<<<< HEAD
	
	//['joint_0.0', 'joint_1.0', 'joint_2.0', 'joint_3.0', 'joint_4.0', 'joint_5.0', 'joint_6.0', 'joint_7.0', 'joint_8.0', 'joint_9.0', 'joint_10.0', 'joint_11.0', 'joint_12.0', 'joint_13.0', 'joint_14.0', 'joint_15.0']
	
=======

>>>>>>> f52340901b8ee1d7e1cd864848f79032fccea222
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
<<<<<<< HEAD
	for(int i=0; i<16; i++) desire_torque[i] = 0.0;
=======
	for(int i=0; i<16; i++) desired_torque[i] = 0.0;
>>>>>>> f52340901b8ee1d7e1cd864848f79032fccea222

	// Set to false for first iteration of control loop (sets PD control at start pose)
	lIsBegin = false;

	// Start ROS time
	tstart = ros::Time::now();

<<<<<<< HEAD
	// Starts control look, messge pub/subs and all other callbacks
	ros::spin();			

	// Clean shutdown: close files, shutdown node, shutdown can, be polite.
	fclose(fid);
	fclose(fidTime);
=======



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
		canDevice->getJointInfo(current_position, current_torque);
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
>>>>>>> f52340901b8ee1d7e1cd864848f79032fccea222
	nh.shutdown();
	delete canDevice;
	printf("\nBye.\n");
	return 0;
}


