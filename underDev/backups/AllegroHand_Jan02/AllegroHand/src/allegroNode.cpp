/*
 * allegroNode.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: Seungsu KIM
 */
 
 // Add something like, if the time is bigger than nominal (0.003)+tolerance, sprit it out to the terminal. Also, consider dropping the file writing all together and saving bag file ros style.
 //Make a launch file that will launch the bag file saver along with allegro. What else is the launch file good for? time to start integrating the kinect???
 
 
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>

#include "ros/ros.h"
#include "ros/service.h"
#include "ros/service_server.h"
#include "sensor_msgs/JointState.h"

#include "BHand/BHand.h"
#include "controlAllegroHand.h"

#define JOINT_STATE_TOPIC "/allegro/joint_state"
#define JOINT_CMD_TOPIC "/allegro/joint_cmd"

double curr_position[DOF_JOINTS];
double curr_position_pre[DOF_JOINTS];
double curr_velocity[DOF_JOINTS];
double curr_torque[DOF_JOINTS];
double desire_position[DOF_JOINTS];
double desire_torque[DOF_JOINTS];

double secs;

double out_torque[DOF_JOINTS];

int frame = 0;
int lEmergencyStop = 0;

double dt;

bool lIsBegin = false;
int kill = false;	

FILE *fid;
FILE *fidTime;

boost::mutex *mutex;

ros::Publisher joint_state_pub;
ros::Subscriber joint_cmd_sub;
sensor_msgs::JointState msgJoint;

ros::Time tstart;
ros::Time tnow;

//	ros::NodeHandle nh;

// initialize BHand 
eMotionType gMotionType = eMotionType_NONE ;
BHand lBHand(eHandType_Right);
	
controlAllegroHand *canDevice;

void SetjointCallback(const sensor_msgs::JointState& msg)
{
	// TODO check joint limits


	// copy
	mutex->lock();
	for(int i=0;i<DOF_JOINTS;i++) desire_position[i] = msg.position[i];
	mutex->unlock();
	//desire_position.Print();
	printf("%f",desire_position[0]);
}

void timerCallback(const ros::TimerEvent& event)
{
	//fid = fopen("./allegro_log.txt", "w+");
	//fidTime = fopen("./allegro_Timelog.txt", "w+");

		//printf("Timer Callback\n\n");
		tnow = ros::Time::now();
		dt = (tnow - tstart).nsec; //1e-9*(tnow - tstart).nsec;
		tstart = tnow;
		
		// save last joint position for velocity calc
		for(int i=0; i<DOF_JOINTS; i++) curr_position_pre[i] = curr_position[i];
		
		//// CAN Communication
		canDevice->setTorque(desire_torque);
		lEmergencyStop = canDevice->update();
		canDevice->getJointInfo(curr_position, curr_torque);
		//// END CAN Communication
				
		if( lEmergencyStop < 0 )
		{
			// stop program when Allegro Hand is switched off
			std::cout << "emergency stop " << std::endl;
			kill = true;
		}
		
		// print to file (doesnt work)
		for(int i=0; i<DOF_JOINTS; i++ ) fprintf(fid, "%lf ", desire_position[i]);
		for(int i=0; i<DOF_JOINTS; i++ ) fprintf(fid, "%lf ", curr_position[i]);
		for(int i=0; i<DOF_JOINTS; i++ ) fprintf(fid, "%lf ", desire_torque[i]);
		fprintf(fid, "%lf\n",dt);
		

		// compute torque using Bhand library
		lBHand.SetJointPosition(curr_position);
		if( lIsBegin == false ){
		// set desired joint position to initial position when program is initialized
			if(frame > 10){
				mutex->lock();
				for(int i=0; i<DOF_JOINTS; i++) desire_position[i] = curr_position[i];
				mutex->unlock();

				lIsBegin = true;
			}
			// start joint position control (Bhand)
			lBHand.SetMotionType(eMotionType_JOINT_PD);
			lBHand.UpdateControl((double)frame*ALLEGRO_CONTROL_TIME_INTERVAL);
			for(int i=0; i<DOF_JOINTS; i++) desire_torque[i] = 0.0;
		}
		else{
			//// Communication with BHAND LIBRARY
			// desired position is either commanded to the joint state subscriber
			// or maintatined as the initial position from program start
			mutex->lock();
			// desired position sent to Bhand lib
			lBHand.SetJointDesiredPosition(desire_position);
			mutex->unlock();

			lBHand.SetMotionType(eMotionType_JOINT_PD);
			lBHand.UpdateControl((double)frame*ALLEGRO_CONTROL_TIME_INTERVAL);
			// necessary torque obtained from Bhand lib
			lBHand.GetJointTorque(out_torque);
			for(int i=0; i<DOF_JOINTS; i++) desire_torque[i] = out_torque[i];
			//// END Communication with BHAND LIBRARY
			
			// calculate current velocity
			for(int i=0; i<DOF_JOINTS; i++) curr_velocity[i] = (curr_position[i] - curr_position_pre[i])/dt;
			
			// current position, velocity and effort (torque) published
			msgJoint.header.stamp = tnow;
			for(int i=0; i<DOF_JOINTS; i++) msgJoint.position[i] = curr_position[i];
			for(int i=0; i<DOF_JOINTS; i++) msgJoint.velocity[i] = curr_velocity[i];
			for(int i=0; i<DOF_JOINTS; i++) msgJoint.effort[i] = desire_torque[i];
			joint_state_pub.publish(msgJoint);
		}
		frame++;

		//ros::spinOnce();
		//rate.sleep();
}

int main(int argc, char** argv)
{
	fid = fopen("./allegro_log.txt", "w+");
	fidTime = fopen("./allegro_Timelog.txt", "w+");

	ros::init(argc, argv, "allegro");
	ros::Time::init();
	
	ros::NodeHandle nh;

	ros::Timer timer = nh.createTimer(ros::Duration(0.003), timerCallback);

	mutex = new boost::mutex();

	joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
	joint_cmd_sub = nh.subscribe(JOINT_CMD_TOPIC, 3, SetjointCallback);
	msgJoint.position.resize(DOF_JOINTS);
	msgJoint.velocity.resize(DOF_JOINTS);
	msgJoint.effort.resize(DOF_JOINTS);
	msgJoint.name.resize(DOF_JOINTS);
	
	msgJoint.name[0]  = "00";
	msgJoint.name[1]  = "01";
	msgJoint.name[2]  = "02";
	msgJoint.name[3]  = "03";
	msgJoint.name[4]  = "10";
	msgJoint.name[5]  = "11";
	msgJoint.name[6]  = "12";
	msgJoint.name[7]  = "13";
	msgJoint.name[8]  = "20";
	msgJoint.name[9]  = "21";
	msgJoint.name[10] = "22";
	msgJoint.name[11] = "23";
	msgJoint.name[12] = "30";
	msgJoint.name[13] = "31";
	msgJoint.name[14] = "32";
	msgJoint.name[15] = "33";

	// initialize BHand controller
	//BHand lBHand(eHandType_Right);
	lBHand.SetTimeInterval(ALLEGRO_CONTROL_TIME_INTERVAL);

	//ros::Rate rate(1./ALLEGRO_CONTROL_TIME_INTERVAL);

	//controlAllegroHand *canDevice;
	// initialize device

	printf("CAN open1\n\n");
	canDevice = new controlAllegroHand();
	printf("CAN open2\n\n");
	canDevice->init();
	printf("CAN open3\n\n");
	usleep(3000);
	
	printf("CAN open4\n\n");

	for(int i=0; i<16; i++) desire_torque[i] = 0.0;

	tstart = ros::Time::now();

	lIsBegin = false;

	//while(kill==false)
	//{
	//}
	
	while(kill==false)
	{
			ros::spinOnce();
	}
			

//if (kill==true)
//{
	fclose(fid);
	fclose(fidTime);
	nh.shutdown();
	delete canDevice;
	printf("bye\n");

	return 0;
//}
}


