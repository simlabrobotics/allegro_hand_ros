/*
 * allegroNode.cpp
 *
 *  Created on: Jan 18, 2013
 *  Authors: Alex Alspach <alexalspach@simlab.co.kr>, Seungsu Kim
 *
 */
 
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>

#include "ros/ros.h"
#include "ros/service.h"
#include "ros/service_server.h"
#include "sensor_msgs/JointState.h"

#include <stdio.h>
#include <iostream>
#include <string>

#include "controlAllegroHand.h"




// Joint Read and Write (Publishers and Subscribers)
#define JOINT_WRITE_TORQUE_TOPIC "/allegroHand/joint_torques"
#define JOINT_READ_ENCODER_TOPIC "/allegroHand/joint_states"

double current_position[DOF_JOINTS];
double previous_position[DOF_JOINTS];
double current_velocity[DOF_JOINTS];
double control_torque[DOF_JOINTS];

sensor_msgs::JointState msgJoint;
ros::Publisher current_jointState_pub;
ros::Subscriber control_torque_sub;

// Time
double atTheBeginningOfTime;
double currentTime;
double dt;
float minutes;

ros::Time tstart;
ros::Time tnow;

// CAN frame counter
int frame = 0;

// Flag to exit when CAN comm. is lost
int lEmergencyStop = 0; 

// Threading
boost::mutex *mutex;

// Initialize CAN device		
controlAllegroHand *canDevice;






void SetTorqueCallback(const sensor_msgs::JointState& msg)
{
	
	// effort = torque (N*m)
	mutex->lock();
	for(int i=0;i<DOF_JOINTS;i++) control_torque[i] = msg.effort[i];
	mutex->unlock();
	
	//printf("Torque Received\n\n");

}






void timerCallback(const ros::TimerEvent& event)
{
	// Calculate loop time;
	tnow = ros::Time::now();
	dt = 1e-9*(tnow - tstart).nsec; //(tnow - tstart).nsec; 
	currentTime+=dt;
	tstart = tnow;

	// Store previous position for velocity calculation
	for(int i=0; i<DOF_JOINTS; i++)		previous_position[i] = current_position[i];

	// CAN Communication
	canDevice->setTorque(control_torque);
	lEmergencyStop = canDevice->update();
	canDevice->getJointInfo(current_position);


	// Stop program when Allegro Hand is switched off
	if( lEmergencyStop < 0 )
	{
		printf("\n\n\nEMERGENCY STOP.\n\n");
		ros::shutdown();
	}
	

	// Construct joint state message (time, names, joint pos, vel, torque)	
	msgJoint.header.stamp = tnow;

	// Calculate joint velocity
	for(int i=0; i<DOF_JOINTS; i++) current_velocity[i] = (current_position[i] - previous_position[i])/dt;
	for(int i=0; i<DOF_JOINTS; i++) msgJoint.velocity[i] = current_velocity[i];

	for(int i=0; i<DOF_JOINTS; i++)		msgJoint.position[i]= current_position[i];
	for(int i=0; i<DOF_JOINTS; i++)		msgJoint.effort[i]	= control_torque[i];	// no torque feedback
	
	// Publist joint state message to be read by controller
	current_jointState_pub.publish(msgJoint);	

	frame++;

} // end timerCallback












int main(int argc, char** argv)
{
	// init ROS Node and time
	ros::init(argc, argv, "allegroCAN");
	ros::Time::init();
	
	ros::NodeHandle nh;

	// Set up timer interrupt and callback
	ros::Timer timer = nh.createTimer(ros::Duration(ALLEGRO_CONTROL_TIME_INTERVAL), timerCallback); // 0.003s (333Hz)

	mutex = new boost::mutex();

	// Publister and Subscriber
	current_jointState_pub	= nh.advertise<sensor_msgs::JointState>(JOINT_READ_ENCODER_TOPIC, 1);
	control_torque_sub	 	= nh.subscribe(JOINT_WRITE_TORQUE_TOPIC, 1, SetTorqueCallback);

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

	// Initialize CAN device
	canDevice = new controlAllegroHand();
	canDevice->init();
	usleep(3000);

	// Initialize torque at zero
	for(int i=0; i<16; i++) control_torque[i] = 0.0;

	// Start ROS time
	atTheBeginningOfTime = ros::Time::now().nsec;
	currentTime = 0.0;
	minutes = 0.0;
	tstart = ros::Time::now();



	// Starts control look, messge pub/subs and all other callbacks
	ros::spin();			




	// Clean shutdown: close files, shutdown node,
	// shutdown can and be polite.
	nh.shutdown();
	delete canDevice;
	printf("\nBye.\n");
	return 0;
	
}











