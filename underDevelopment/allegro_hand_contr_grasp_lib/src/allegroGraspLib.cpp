/*
 * allegroNode.cpp
 *
 *  Created on: Nov 14, 2012
 *  Authors: Alex Alspach, Seungsu Kim
 */
 
#include <iostream>

// Necessary?
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
// =========

#include "ros/ros.h"
#include "ros/service.h"
#include "ros/service_server.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <iostream>
#include <string>

#include "BHand/BHand.h"

#define DOF_JOINTS 16
#define ALLEGRO_CONTROL_TIME_INTERVAL 0.003



#define JOINT_WRITE_TORQUE_TOPIC "/allegroHand/joint_torques"
#define JOINT_READ_ENCODER_TOPIC "/allegroHand/joint_states"
#define GRASP_CMD_TOPIC "/allegroHand/grasp_cmd" // ADDED CAPITAL H - BEWARE
#define JOINT_POS_CMD_TOPIC "/allegroHand/joint_pos_cmd"



double current_position[DOF_JOINTS];
double desired_position[DOF_JOINTS];
double control_torque[DOF_JOINTS];
std::string  grasp_cmd;

boost::mutex *mutex;
int frame = 0;

// Flags
bool pdControl = true;
bool lIsBegin = false;

// ROS Messages
sensor_msgs::JointState msgJoint;
ros::Publisher control_torque_pub;
ros::Subscriber current_jointState_sub; //switched - opposite of allegroHand_core
ros::Subscriber grasp_cmd_sub;
ros::Subscriber joint_pos_cmd_sub;

// ROS Time
ros::Time tstart;
ros::Time tnow;

// Initialize BHand 
eMotionType gMotionType = eMotionType_NONE ;
BHand lBHand(eHandType_Left); //need to make this an option externally w/o recompiling
//BHand lBHand(eHandType_Right);


void jointPosCmdCallback(const sensor_msgs::JointState& msg)
{

}

void setCurrentPosCallback(const sensor_msgs::JointState& msg)
{
	// TODO check joint limits
	pdControl=true;
    //lBHand.SetMotionType(eMotionType_JOINT_PD);
	mutex->lock();
	for(int i=0;i<DOF_JOINTS;i++) current_position[i] = msg.position[i];
	mutex->unlock();
	//printf("Set to PD Control mode to execute Joint Pos. command");
	//printf("%f",desire_position[0]);
	lBHand.SetMotionType(eMotionType_JOINT_PD);

}

void libCmdCallback(const std_msgs::String::ConstPtr& msg)
{

}




int main(int argc, char** argv)
{
	ros::init(argc, argv, "allegro_graspLib");
	ros::Time::init();
	
		//ros::Rate rate((1./ALLEGRO_CONTROL_TIME_INTERVAL));
ros::Rate rate((300.0));

	ros::NodeHandle nh;

	mutex = new boost::mutex();
	
	
	
	// Publister and Subscriber
// Sends command torque to CAN (allegroHand_core)
	control_torque_pub	 	= nh.advertise<sensor_msgs::JointState>(JOINT_WRITE_TORQUE_TOPIC, 3);
// Obtains current joint position from CAN (allegroHand_core)
	current_jointState_sub	= nh.subscribe(JOINT_READ_ENCODER_TOPIC, 3, setCurrentPosCallback);

// Obtains grasp-type command from user or other command publisher
	grasp_cmd_sub 			= nh.subscribe(GRASP_CMD_TOPIC, 1000, libCmdCallback);
// Obtains PD joint positon command from user or other command publisher
	joint_pos_cmd_sub 		= nh.subscribe(JOINT_POS_CMD_TOPIC, 1000, jointPosCmdCallback);



	// Create arrays 16 long for each of the four joint state components
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
	
	
	//	lIsBegin = false;
	
	
	for(int i=0; i<16; i++) control_torque[i] = 0.0;
	

	
	// Start ROS time
	tstart = ros::Time::now();


		ros::spinOnce();
	usleep(1000000); // to make sure a current position value is cued in subscriber

//////////////////////////////////////////////////////////////////////////////////////////////


	while(ros::ok())
	{

		//tend = ros::Time::now();
		//dt = 1e-9*(tend - tstart).nsec;
		//tstart = tend;
		ros::spinOnce();
		
		
		
	
		

		// get positions
		//canDevice->getJointInfo(current_position, curr_torque);

		//if( lEmergencyStop < 0 )
		//{
		//	std::cout << "emergency stop " << std::endl;
		//	break;
		//}

		// compute torque
		lBHand.SetJointPosition(current_position);
		if( lIsBegin == false ){
			if(frame > 10){
				mutex->lock();
				for(int i=0; i<DOF_JOINTS; i++) desired_position[i] = current_position[i];
				mutex->unlock();

				//for(int i=0; i<DOF_JOINTS; i++) printf("%f \t", current_position[i]);

				lIsBegin = true;
			}

			lBHand.SetMotionType(eMotionType_JOINT_PD);
			lBHand.UpdateControl((double)frame*ALLEGRO_CONTROL_TIME_INTERVAL);
			for(int i=0; i<DOF_JOINTS; i++) control_torque[i] = 0.0;
		}
		else{
		
		//for(int i=0; i<DOF_JOINTS; i++) printf("%f \t", current_position[i]);
		
			mutex->lock();
			lBHand.SetJointDesiredPosition(desired_position);
			mutex->unlock();

			lBHand.SetMotionType(eMotionType_JOINT_PD);

			lBHand.UpdateControl((double)frame*ALLEGRO_CONTROL_TIME_INTERVAL);
			lBHand.GetJointTorque(control_torque);

			//for(int i=0; i<DOF_JOINTS; i++) control_torque[i] = out_torque[i];
			for(int i=0; i<DOF_JOINTS; i++) msgJoint.effort[i] = control_torque[i];
			//joint_state_pub.publish(msgJoint);
		}

		
				// publish torque
		//canDevice->setTorque(control_torque);
		for(int i=0; i<DOF_JOINTS; i++)		msgJoint.effort[i]	= control_torque[i];
		control_torque_pub.publish(msgJoint);
		frame++;

		rate.sleep();
	}


//////////////////////////////////////////////////////////////////////////////////////////////






	// Starts control look, messge pub/subs and all other callbacks
	ros::spin();	

	// Clean shutdown: close files, shutdown node, shutdown can, be polite.
	nh.shutdown();
	printf("\nBye.\n");
	return 0;
}



































