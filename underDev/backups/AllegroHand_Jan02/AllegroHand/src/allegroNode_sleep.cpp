/*
 * allegroNode.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: Seungsu KIM
 */
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
double curr_torque[DOF_JOINTS];
double desire_position[DOF_JOINTS];
double desire_torque[DOF_JOINTS];

double out_torque[DOF_JOINTS];


boost::mutex *mutex;
ros::Publisher joint_state_pub;
ros::Subscriber joint_cmd_sub;
sensor_msgs::JointState msgJoint;


eMotionType gMotionType = eMotionType_NONE ;

ros::Time tstart;
ros::Time tend;


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


int main(int argc, char** argv)
{
	ros::init(argc, argv, "allegro");
	ros::Time::init();
	ros::NodeHandle nh;
	int lEmergencyStop = 0;

	mutex = new boost::mutex();

	joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
	joint_cmd_sub = nh.subscribe(JOINT_CMD_TOPIC, 3, SetjointCallback);
	msgJoint.position.resize(DOF_JOINTS);


	// initialize BHand controller
	BHand lBHand(eHandType_Right);
	lBHand.SetTimeInterval(ALLEGRO_CONTROL_TIME_INTERVAL);

	ros::Rate rate(1./ALLEGRO_CONTROL_TIME_INTERVAL);
	int frame =0;

	// initialize device
	controlAllegroHand *canDevice;
	canDevice = new controlAllegroHand();
	canDevice->init();
	usleep(3000);

	for(int i=0; i<16; i++) desire_torque[i] = 0.0;

	tstart = ros::Time::now();
	double dt;

	bool lIsBegin = false;
	FILE *fid;
	fid = fopen("./allegro_log.txt", "w+");
	
	FILE *fidTime;
	fidTime = fopen("./allegro_Timelog.txt", "w+");

	while(ros::ok())
	{

		tend = ros::Time::now();
		dt = 1e-9*(tend - tstart).nsec;
		tstart = tend;

		canDevice->setTorque(desire_torque);
		lEmergencyStop = canDevice->update();
		canDevice->getJointInfo(curr_position, curr_torque);




		for(int i=0; i<DOF_JOINTS; i++ ) fprintf(fid, "%lf ", desire_position[i]);
		for(int i=0; i<DOF_JOINTS; i++ ) fprintf(fid, "%lf ", curr_position[i]);
		for(int i=0; i<DOF_JOINTS; i++ ) fprintf(fid, "%lf ", desire_torque[i]);
		fprintf(fid, "\n");
		
		double secs =ros::Time::now().toSec();
		fprintf(fidTime, "%lf ", secs);
		fprintf(fidTime, "\n");

		if( lEmergencyStop < 0 )
		{
			std::cout << "emergency stop " << std::endl;
			break;
		}

		// compute torque
		lBHand.SetJointPosition(curr_position);
		if( lIsBegin == false ){
			if(frame > 10){
				mutex->lock();
				for(int i=0; i<DOF_JOINTS; i++) desire_position[i] = curr_position[i];
				mutex->unlock();

				lIsBegin = true;
			}

			lBHand.SetMotionType(eMotionType_JOINT_PD);
			lBHand.UpdateControl((double)frame*ALLEGRO_CONTROL_TIME_INTERVAL);
			for(int i=0; i<DOF_JOINTS; i++) desire_torque[i] = 0.0;
		}
		else{
			mutex->lock();
			lBHand.SetJointDesiredPosition(desire_position);
			mutex->unlock();

			lBHand.SetMotionType(eMotionType_JOINT_PD);

			lBHand.UpdateControl((double)frame*ALLEGRO_CONTROL_TIME_INTERVAL);
			lBHand.GetJointTorque(out_torque);

			for(int i=0; i<DOF_JOINTS; i++) desire_torque[i] = out_torque[i];
			for(int i=0; i<DOF_JOINTS; i++) msgJoint.position[i] = curr_position[i];
			joint_state_pub.publish(msgJoint);
		}
		frame++;

		ros::spinOnce();
		rate.sleep();
	}

	fclose(fid);
	nh.shutdown();
	delete canDevice;
	printf("bye\n");

	return 0;
}


